import math
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pynput import keyboard

class Drone_Controller:
    def __init__(self):
        try:
            # 1. CONNESSIONE ALLA REMOTE API (ZMQ)
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            
            # Congela il tempo di CoppeliaSim e lo fa avanzare solo su comando di Python.
            # Questo garantisce una frequenza di campionamento del controllo cristallina a 50Hz.
            self.sim.setStepping(True)
            
            # Recupero dell'oggetto drone nella scena di CoppeliaSim
            self.drone = self.sim.getObject('/Quadcopter_target')
            
            # 2. AGGIUNTA TARGET GENERICO
            try:
                self.target_dummy = self.sim.getObject('/Target')
                print("[INFO] Target Dummy trovato.")
            except:
                self.target_dummy = None
                print("[WARNING] Dummy 'Target' non trovato. Consentito solo l'uso dei comandi manuali.")

            # 3. RESET POSIZIONE E CAMERA
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05))
            self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, 0.0))
            
            # Inizializzazione segnale camera (0 gradi)
            self.camera_tilt = 0.0
            self.sim.setFloatSignal("cameraTilt", self.camera_tilt)

            try:
                self.vision_sensor = self.sim.getObject('/visionSensor')
                self.camera_fov = math.radians(60)
                self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
            except Exception as e:
                print(f"[WARNING] Impossibile configurare /visioneSensor: {e}")
            
            print("--- DJI Mini 3: Sistema di Navigazione GNSS Ready ---")
        except Exception as e:
            print(f"Errore inizializzazione: {e}")
            exit()

        # --- PARAMETRI DI NAVIGAZIONE E GEODETICA ---
        self.ref_lat, self.ref_lon = 0.0, 0.0
        self.LAT_METERS_PER_DEG = 111319.9
        
        # CONTROLLORE PD (PROPORZIONALE-DERIVATIVO) ---
        # Parametri di guadagno per il cappio di controllo dello spostamento lineare
        self.KP_MOVE = 0.18               # Reattività sulla distanza
        self.KD_MOVE = 0.04               # Smorzamento della frenata lineare (evita overshooting)
        
        # Parametri di guadagno per il cappio di controllo dell'orientamento (Yaw)
        self.KP_YAW = 0.50                # Reattività angolare
        self.KD_YAW = 0.12                # Smorzamento derivativo (annulla lo sfarfallio e l'effetto pendolo)
        
        # Memorie storiche per il calcolo delle derivate temporali istantanee
        self.last_dist_error = 0.0
        self.last_yaw_error = 0.0
        
        # SATURAZIONI CINEMATICHE DI SICUREZZA
        self.MAX_STEP_MOVE = 0.12         
        self.MIN_STEP_MOVE = 0.005        
        self.MAX_STEP_YAW = math.radians(6.0) 
        
        # Parametri manuali standard
        self.STEP_MOVE = 0.08            
        self.STEP_YAW = math.radians(4)   
        self.TILT_STEP = math.radians(2)  
        
        # Soglia di aggancio Waypoint millimetrica (ridotta a 10cm grazie alla stabilità del PD)
        self.ARRIVED_THRESH = 0.10

        # Stati operativi
        self.takeoff_mode = False
        self.takeoff_target_alt = 0.8     
        self.takeoff_speed = 0.5          

        self.land_mode = False
        self.land_speed = 0.3             
        
        self.orbit_mode = False
        self.orbit_center = [0, 0, 1]     
        self.orbit_radius = 0.5           
        self.orbit_angle = 0.0            
        self.orbit_speed = 0.05           

        self.running = True
        self.waypoint_mode = False       
        self.is_airborne = False          
        self.key_pressed = None          

    def get_telemetry(self):
        """Calcola e restituisce i dati di volo e camera in tempo reale"""
        try:
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
            
            lat = self.ref_lat + (pos[0] / self.LAT_METERS_PER_DEG)
            lon = self.ref_lon + (pos[1] / (self.LAT_METERS_PER_DEG * math.cos(math.radians(lat))))
            
            roll_deg = math.degrees(ori[0])
            pitch_deg = math.degrees(ori[1])
            yaw_deg = math.degrees(ori[2])

            if abs(roll_deg) < 0.05: roll_deg = 0.0
            if abs(pitch_deg) < 0.05: pitch_deg = 0.0
            if abs(yaw_deg) < 0.05: yaw_deg = 0.0
            
            return (f"lat: {lat:.7f} | lon: {lon:.7f} | alt: {pos[2]:.2f}m | "
                    f"yaw: {yaw_deg:.1f}° pitch: {pitch_deg:.1f}° roll: {roll_deg:.1f}°")
        except:
            return None

    def gradual_takeoff(self):
        """Gestisce la salita graduale e la stabilizzazione alla quota target"""
        if not self.takeoff_mode: return
        try:
            drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            current_alt = drone_pos[2]
            dz = self.takeoff_target_alt - current_alt

            if abs(dz) < 0.01:
                self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], self.takeoff_target_alt))
                self.takeoff_mode = False
                self.is_airborne = True 
                print(f"\n[TAKEOFF] Completato. Drone stabilizzato a {self.takeoff_target_alt}m.")
                return

            step = math.copysign(self.takeoff_speed * 0.02, dz) # Adattato per il ciclo sincrono a 50Hz
            if abs(step) > abs(dz): step = dz

            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], current_alt + step))
        except:
            self.takeoff_mode = False
    
    def gradual_landing(self):
        """Gestisce la discesa graduale controllata fino al touchdown sul terreno"""
        if not self.land_mode: return
        try:
            drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            current_alt = drone_pos[2]
            
            if current_alt <= 0.052:
                self.land_mode = False
                self.is_airborne = False 
                self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], 0.05))
                print(f"\n[LANDING] Atterraggio completato con successo. Drone posato a terra.")
                return

            velocita_effettiva = self.land_speed
            SOGLIA_RALLENTAMENTO = 0.25
            
            if current_alt <= SOGLIA_RALLENTAMENTO:
                rapporto = current_alt / SOGLIA_RALLENTAMENTO
                velocita_effettiva = max(self.land_speed * rapporto, 0.04)

            step = velocita_effettiva * 0.02
            nuova_alt = current_alt - step
            if nuova_alt < 0.05: nuova_alt = 0.05

            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], nuova_alt))
        except:
            self.land_mode = False

    def update_orbit(self):
        """Calcola la posizione circolare (senso orario) e punta il drone verso il centro"""
        if not self.orbit_mode: return
        
        self.orbit_angle -= self.orbit_speed * 0.4 # Scalato per stepping sincrono
        new_x = self.orbit_center[0] + self.orbit_radius * math.cos(self.orbit_angle)
        new_y = self.orbit_center[1] + self.orbit_radius * math.sin(self.orbit_angle)
        target_yaw = self.orbit_angle + math.pi

        self.sim.setObjectPosition(self.drone, self.sim.handle_world, (new_x, new_y, self.orbit_center[2]))
        self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, target_yaw))

    def follow_target(self):
        """
        Navigazione Autonoma Waypoint con Regolazione ad Anello Chiuso PD (Proporzionale-Derivativo).
        Calcola l'azione predittiva sulla variazione dell'errore per annullare sbandate aerodinamiche
        e oscillazioni parassite, raccordando perfettamente la traiettoria sul target.
        """
        if not self.target_dummy: return
        try: 
            # 1. Acquisizione degli stati cartesiani globali
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            target_pos = self.sim.getObjectPosition(self.target_dummy, self.sim.handle_world)
            
            error_x = target_pos[0] - pos[0]
            error_y = target_pos[1] - pos[1]
            
            # Calcolo Errore Lineare (Distanza Euclidea istantanea)
            dist_error = math.sqrt(error_x**2 + error_y**2)
            if dist_error < self.ARRIVED_THRESH:
                self.waypoint_mode = False
                self.last_dist_error = 0.0
                self.last_yaw_error = 0.0
                print("\n[AUTO] Waypoint Intercettato. Stabilizzazione ad anello chiuso completata!")
                return

            # Quota di volo controllata (Asse Z globale)
            err_alt = target_pos[2] - pos[2]
            move_z = 0.0
            if abs(err_alt) > 0.5:
                move_z = math.copysign(0.04, err_alt)

            # 2. CALCOLO ERRORE ORIENTAMENTO (YAW) CON STRUTTURA ANTIAVVOLGIMENTO
            angolo_linea_diagonale = math.atan2(error_y, error_x)
            angolo_puntamento_target = angolo_linea_diagonale - (math.pi / 2) # Allineamento muso su Y locale

            ori_attuale = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
            yaw_attuale = ori_attuale[2]

            yaw_error = angolo_puntamento_target - yaw_attuale
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error)) # Normalizzazione tra -pi e +pi

            # --- CALCOLO AZIONE DERIVATIVA (D) ---
            # Misura la velocità di variazione dell'errore per contrastare l'inerzia prima che si verifichi lo sbandamento
            derivative_move = dist_error - self.last_dist_error if self.last_dist_error != 0.0 else 0.0
            derivative_yaw = yaw_error - self.last_yaw_error if self.last_yaw_error != 0.0 else 0.0
            
            # Aggiornamento memorie storiche dei registri
            self.last_dist_error = dist_error
            self.last_yaw_error = yaw_error

            # 3. LEGGE DI CONTROLLO PD PER LO YAW
            step_yaw_dinamico = (yaw_error * self.KP_YAW) + (derivative_yaw * self.KD_YAW)
            
            # Saturazione di stabilità dello Yaw
            if abs(step_yaw_dinamico) > self.MAX_STEP_YAW:
                step_yaw_dinamico = math.copysign(self.MAX_STEP_YAW, step_yaw_dinamico)
            
            # Esecuzione della rotazione accoppiata
            self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, step_yaw_dinamico))

            # 4. LEGGE DI CONTROLLO PD PER LO SPOSTAMENTO LINEARE
            passo_avanzamento_teorico = (dist_error * self.KP_MOVE) + (derivative_move * self.KD_MOVE)
            
            # Saturazione fisica della velocità lineare
            if passo_avanzamento_teorico > self.MAX_STEP_MOVE:
                passo_avanzamento_teorico = self.MAX_STEP_MOVE
            elif passo_avanzamento_teorico < self.MIN_STEP_MOVE:
                passo_avanzamento_teorico = self.MIN_STEP_MOVE

            # Funzione di Smorzamento di Prua (Raccordo Dinamico tra traiettoria e allineamento)
            # Se la prua diverge molto dall'obiettivo, rallenta bruscamente la traslazione per stringere la curva.
            allineamento_fronte = math.cos(yaw_error)
            if allineamento_fronte < 0.0: allineamento_fronte = 0.0
                
            step_move_dinamico = passo_avanzamento_teorico * allineamento_fronte

            # Avanzamento raccordato sull'asse Y locale del drone
            self.sim.setObjectPosition(self.drone, self.drone, (0, step_move_dinamico, 0))

            # Correzione quota globale indipendente
            if move_z != 0.0:
                pos_corrente = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                self.sim.setObjectPosition(self.drone, self.sim.handle_world, (pos_corrente[0], pos_corrente[1], pos_corrente[2] + move_z))

        except Exception as e:
            print(f"\n[ERROR PD WAYPOINT]: {e}")

    def on_press(self, key): 
        self.key_pressed = key   

    def on_release(self, key):
        self.key_pressed = None 

    def process_input(self):
        """Gestione dei comandi inclusa la rotazione della fotocamera"""
        if not self.key_pressed: return 
        k = self.key_pressed
        
        try:
            if hasattr(k, 'char'):
                if k.char not in ['t', 'q'] and not self.is_airborne: return
                
                if k.char == 'w':
                    self.takeoff_mode = False 
                    self.land_mode = False 
                    self.sim.setObjectPosition(self.drone, self.drone, (0, 0, self.STEP_MOVE))

                elif k.char == 's': 
                    self.takeoff_mode = False
                    self.land_mode = False 
                    drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                    
                    if drone_pos[2] > 0.20:
                        nuva_alt_manuale = drone_pos[2] - self.STEP_MOVE
                        if nuva_alt_manuale < 0.20:
                            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], 0.20))
                        else:
                            self.sim.setObjectPosition(self.drone, self.drone, (0, 0, -self.STEP_MOVE))
                
                elif k.char == 't': 
                    self.orbit_mode = False
                    self.waypoint_mode = False 
                    self.land_mode = False
                    self.takeoff_mode = True
                    self.key_pressed = None

                elif k.char == 'g':
                    self.takeoff_mode = False
                    self.orbit_mode = False 
                    self.waypoint_mode = False 
                    self.land_mode = True
                    self.key_pressed = None
                
                elif k.char == 'a': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, self.STEP_YAW))
                elif k.char == 'd': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, -self.STEP_YAW))
                
                elif k.char == 'o':
                    self.takeoff_mode = False
                    self.land_mode = False
                    self.orbit_mode = not self.orbit_mode
                    if self.orbit_mode:
                        pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                        self.orbit_center = [pos[0], pos[1], pos[2]]
                        self.orbit_angle = 0
                        print(f"\n[ORBIT] ON - Centro: {self.orbit_center}")
                    self.key_pressed = None

                elif k.char == 'i': 
                    self.camera_tilt += self.TILT_STEP
                    if self.camera_tilt > math.radians(60): self.camera_tilt = math.radians(60)
                    self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
                
                elif k.char == 'k': 
                    self.camera_tilt -= self.TILT_STEP
                    if self.camera_tilt < math.radians(-90): self.camera_tilt = math.radians(-90)
                    self.sim.setFloatSignal("cameraTilt", self.camera_tilt)

                elif k.char == 'u': 
                    self.camera_fov -= math.radians(2)
                    if self.camera_fov < math.radians(10): self.camera_fov = math.radians(10)
                    self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
                elif k.char == 'j': 
                    self.camera_fov += math.radians(2)
                    if self.camera_fov > math.radians(100): self.camera_fov = math.radians(100)
                    self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)

                elif k.char == 'p':
                    self.takeoff_mode = False
                    self.land_mode = False 
                    self.waypoint_mode = not self.waypoint_mode
                    self.last_dist_error = 0.0
                    self.last_yaw_error = 0.0
                    print(f"\n[AUTO] Controllore PD Traiettoria: {'ATTIVO' if self.waypoint_mode else 'DISATTIVO'}")
                    self.key_pressed = None
                
                elif k.char == 'q': self.running = False
            
            if k == keyboard.Key.up: self.sim.setObjectPosition(self.drone, self.drone, (0, self.STEP_MOVE, 0))
            elif k == keyboard.Key.down: self.sim.setObjectPosition(self.drone, self.drone, (0, -self.STEP_MOVE, 0))
            elif k == keyboard.Key.left: self.sim.setObjectPosition(self.drone, self.drone, (-self.STEP_MOVE, 0, 0))
            elif k == keyboard.Key.right: self.sim.setObjectPosition(self.drone, self.drone, (self.STEP_MOVE, 0, 0))
        except:
            pass

    def run(self):
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        
        while self.running:
            self.process_input()

            if self.takeoff_mode: self.gradual_takeoff()
            elif self.land_mode: self.gradual_landing()

            if self.orbit_mode: self.update_orbit()
            elif self.waypoint_mode: self.follow_target()

            line = self.get_telemetry()
            if line:
                if self.takeoff_mode: mode = "[TAKEOFF]"
                elif self.land_mode: mode = "[LANDING]"
                elif self.orbit_mode: mode = "[ORBITING]"
                elif self.waypoint_mode: mode = "[FOLLOWING]"
                else: mode = "[MANUAL]"
                print(f"\r{line} {mode}", end="", flush=True)
            
            # avanzando di un fotogramma si mantiene il campionamento bloccato e pulito indipendentemente dalla CPU.
            self.sim.step()
            time.sleep(0.01) # Mantiene la responsività del thread della tastiera
            
        listener.stop()
        # Ripristina la simulazione asincrona classica alla chiusura per non bloccare la UI di CoppeliaSim
        self.sim.setStepping(False)
        print("\nDisconnessione completata.")

if __name__ == "__main__":
    controller = Drone_Controller()
    controller.run()