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
            
            # Impostazione yaw iniziale a 0 radianti
            self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, 0.0))
            
            # Inizializzazione segnale camera (0 gradi)
            self.camera_tilt = 0.0
            self.sim.setFloatSignal("cameraTilt", self.camera_tilt)

            try:
                self.vision_sensor = self.sim.getObject('/visionSensor')
                self.camera_fov = math.radians(60) # Field of view iniziale di 60°
                self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
            except Exception as e:
                print(f"[WARNING] Impossibile trovare o configurare /visioneSensor: {e}")
            
            print("--- DJI Mini 3: Sistema di Navigazione GNSS Pronto ---")
        except Exception as e:
            print(f"Errore inizializzazione: {e}")
            exit()

        # --- PARAMETRI DI NAVIGAZIONE E GEODETICA ---
        self.ref_lat, self.ref_lon = 0.0, 0.0
        self.LAT_METERS_PER_DEG = 111319.9
        
        # Parametri di volo
        self.STEP_MOVE = 0.08            
        self.STEP_YAW = math.radians(5)  
        self.TILT_STEP = math.radians(2)  # Velocità rotazione camera
        
        # PARAMETRO CRUCIALE: Soglia di arrivo allargata per intercettare il target (25 cm)
        self.ARRIVED_THRESH = 0.25

        # Parametri per il decollo graduale
        self.takeoff_mode = False
        self.takeoff_target_alt = 0.8     # Stabilizzazione a 0.8 m da terra
        self.takeoff_speed = 0.5          # Velocità di salita (metri al secondo)

        self.land_mode = False
        self.land_speed = 0.3             # Velocità di discesa sicura (m/s)
        
        # Parametri orbita
        self.orbit_mode = False
        self.orbit_center = [0, 0, 1]     # Centro dell'orbita
        self.orbit_radius = 0.5           # Raggio in metri
        self.orbit_angle = 0.0            # Angolo corrente
        self.orbit_speed = 0.05           # Velocità angolare 

        self.running = True
        self.waypoint_mode = False       
        self.is_airborne = False          # Inizializzato lo stato di volo
        self.key_pressed = None          

    def get_telemetry(self):
        """Calcola e restituisce i dati di volo e camera in tempo reale"""
        try:
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
            
            # Conversione GNSS
            lat = self.ref_lat + (pos[0] / self.LAT_METERS_PER_DEG)
            lon = self.ref_lon + (pos[1] / (self.LAT_METERS_PER_DEG * math.cos(math.radians(lat))))
            
            # Conversione in gradi ed eliminazione del segno -0.0 causato da float negativi infinitesimali
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

            step = math.copysign(self.takeoff_speed * 0.05, dz)
            if abs(step) > abs(dz):
                step = dz

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

            step = velocita_effettiva * 0.05
            nuova_alt = current_alt - step
            if nuova_alt < 0.05: nuova_alt = 0.05

            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], nuova_alt))
        except:
            self.land_mode = False

    def update_orbit(self):
        """Calcola la posizione circolare (senso orario) e punta il drone verso il centro"""
        if not self.orbit_mode: return
        
        self.orbit_angle -= self.orbit_speed
        new_x = self.orbit_center[0] + self.orbit_radius * math.cos(self.orbit_angle)
        new_y = self.orbit_center[1] + self.orbit_radius * math.sin(self.orbit_angle)
        target_yaw = self.orbit_angle + math.pi

        self.sim.setObjectPosition(self.drone, self.sim.handle_world, (new_x, new_y, self.orbit_center[2]))
        self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, target_yaw))

    def follow_target(self):
        """Inseguimento target GNSS con allineamento e frenata adattiva sulla soglia di arrivo"""
        if not self.target_dummy: return
        try: 
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
            
            cur_lat = self.ref_lat + (pos[0] / self.LAT_METERS_PER_DEG)
            cur_lon = self.ref_lon + (pos[1] / (self.LAT_METERS_PER_DEG * math.cos(math.radians(cur_lat))))
            
            target_pos = self.sim.getObjectPosition(self.target_dummy, self.sim.handle_world)
            end_lat = self.ref_lat + (target_pos[0] / self.LAT_METERS_PER_DEG)
            end_lon = self.ref_lon + (target_pos[1] / (self.LAT_METERS_PER_DEG * math.cos(math.radians(end_lat))))
            
            # Gestione asse Z (Quota con soglia di tolleranza allineata a Kotlin)
            err_alt = target_pos[2] - pos[2]
            move_z = 0.0
            if abs(err_alt) > 0.5:
                move_z = math.copysign(0.04, err_alt)

            # Formula Haversine (Distanza)
            R = 6371000.0  
            dphi = math.radians(end_lat - cur_lat)
            dlam = math.radians(end_lon - cur_lon)
            phi1 = math.radians(cur_lat)
            phi2 = math.radians(end_lat)
            
            a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2)**2
            dist = R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            
            # Controllo di arrivo basato sulla tolleranza 
            if dist < self.ARRIVED_THRESH:
                self.waypoint_mode = False
                print("\n[AUTO] Target Raggiunto con successo!")
                return

            # Formula Bearing (Prua geografica 0..360)
            x = math.sin(dlam) * math.cos(phi2)
            y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
            target_bearing = (math.degrees(math.atan2(x, y)) + 360) % 360
            
            # Compensazione +90° per riallineare lo zero della formula al muso del drone (asse Y+)
            cur_yaw_deg = (math.degrees(ori[2]) + 90 + 360) % 360
            
            err_ang = target_bearing - cur_yaw_deg
            if err_ang > 180: err_ang -= 360
            elif err_ang < -180: err_ang += 360

            # Controllo allineamento sul posto
            ANGLE_TOERANCE = 5.0  
            aligned = False
            
            if abs(err_ang) <= ANGLE_TOERANCE:
                aligned = True
            else:
                step_yaw = math.copysign(math.degrees(self.STEP_YAW), err_ang)
                if abs(step_yaw) > abs(err_ang): step_yaw = err_ang
                nuovo_yaw = math.radians(math.degrees(ori[2]) + step_yaw)
                self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (ori[0], ori[1], nuovo_yaw))

            # Avanzamento assiale puro (Solo se allineato)
            move_x, move_y = 0.0, 0.0
            if aligned:
                # Gestione dinamica della velocità basata sulla distanza (rallenta sotto i 60 cm)
                speed_modifier = self.STEP_MOVE if dist > 0.60 else (self.STEP_MOVE * 0.4)
                
                # Aggiunto l'offset trigonometrico di 90° (math.pi / 2) per proiettare la spinta in avanti lungo il muso reale
                angolo_reale_muso = ori[2] + (math.pi / 2)
                move_x = speed_modifier * math.cos(angolo_reale_muso)
                move_y = speed_modifier * math.sin(angolo_reale_muso)
            
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, 
                                       (pos[0] + move_x, pos[1] + move_y, pos[2] + move_z))
        except Exception as e:
            print(f"\n[ERROR FOLLOW GNSS]: {e}")

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
                    time.sleep(0.2)

                elif k.char == 'g':
                    self.takeoff_mode = False
                    self.orbit_mode = False 
                    self.waypoint_mode = False 
                    self.land_mode = True
                    self.key_pressed = None
                    time.sleep(0.2)
                
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
                    time.sleep(0.2)

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
                    print(f"\n[AUTO] {'ON' if self.waypoint_mode else 'OFF'}")
                    self.key_pressed = None
                    time.sleep(0.2)
                
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
            
            time.sleep(0.05) 
            
        listener.stop()
        print("\nDisconnessione completata.")

if __name__ == "__main__":
    controller = Drone_Controller()
    controller.run()