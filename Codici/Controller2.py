import math
import time
import numpy as np
import cv2 
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
            
            # 2. CONFIGURAZIONE TARGET DUMMY E TELECAMERA
            try:
                self.target_dummy = self.sim.getObject('/Target')
                print("[INFO] Target Dummy trovato.")
            except:
                self.target_dummy = None
                print("[WARNING] Dummy 'Target' non trovato.")

            try:
                self.vision_sensor = self.sim.getObject('/visionSensor')
                self.camera_fov = math.radians(60) 
                self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
                print("[INFO] Vision Sensor trovato e configurato.")
            except:
                self.vision_sensor = None
                print("[WARNING] Impossibile trovare /visionSensor.")
            
            # --- CONFIGURAZIONE DIZIONARIO ARUCO STANDARD 4X4 ---
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            
            # Ottimizzazioni per contrasti netti digitali (CoppeliaSim Spec)
            self.aruco_params.adaptiveThreshWinSizeMin = 3
            self.aruco_params.adaptiveThreshWinSizeMax = 23
            self.aruco_params.adaptiveThreshWinSizeStep = 4
            self.aruco_params.adaptiveThreshConstant = 7
            self.aruco_params.minMarkerPerimeterRate = 0.03 
            
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

            # 3. RESET POSIZIONE E ORIENTAMENTO INIZIALE
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05))
            self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, 0.0))
            
            # Inizializzazione segnale camera (Brandeggio manuale via tastiera)
            self.camera_tilt = 0.0
            self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
            
            print("--- DJI Mini 3: Controller Memoria Visiva con Stop Automatico Ready ---")
        except Exception as e:
            print(f"Errore inizializzazione: {e}")
            exit()

        # --- PARAMETRI DI NAVIGAZIONE E GEODETICA ---
        self.ref_lat, self.ref_lon = 0.0, 0.0
        self.LAT_METERS_PER_DEG = 111319.9
        
        # Parametri di movimento originali
        self.STEP_MOVE = 0.08            
        self.STEP_YAW = math.radians(5)  
        self.TILT_STEP = math.radians(2)  
        self.ARRIVED_THRESH = 0.25

        # --- VARIABILI DI STATO DELLE MODALITÀ ---
        self.waypoint_mode = False        
        self.aruco_mode = False           
        
        # Sequenza Multi-ArUco (Lista impostata con l'ID 0 per il test singolo)
        self.aruco_path = [0]       
        self.current_path_index = 0       
        self.centering_frames_counter = 0 

        # Stati di volo originali
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
        self.is_airborne = False          
        self.key_pressed = None          
        self.current_bearing_lock = None

    def get_telemetry(self):
        try:
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
            lat = self.ref_lat + (pos[0] / self.LAT_METERS_PER_DEG)
            lon = self.ref_lon + (pos[1] / (self.LAT_METERS_PER_DEG * math.cos(math.radians(lat))))
            
            yaw_deg = math.degrees(ori[2])
            if abs(yaw_deg) < 0.05: yaw_deg = 0.0
            
            mode_str = "NONE"
            if self.waypoint_mode: mode_str = "Dummy"
            elif self.aruco_mode: mode_str = f"ArUco ID {self.aruco_path[self.current_path_index] if self.current_path_index < len(self.aruco_path) else 'FINISH'}"

            return f"lat: {lat:.7f} | lon: {lon:.7f} | alt: {pos[2]:.2f}m | yaw: {yaw_deg:.1f}° | Target: {mode_str}"
        except:
            return None

    def gradual_takeoff(self):
        if not self.takeoff_mode: return
        try:
            drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            current_alt = drone_pos[2]
            dz = self.takeoff_target_alt - current_alt
            if abs(dz) < 0.01:
                self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], self.takeoff_target_alt))
                self.takeoff_mode = False
                self.is_airborne = True 
                print(f"\n[TAKEOFF] Completato a {self.takeoff_target_alt}m.")
                return
            step = math.copysign(self.takeoff_speed * 0.05, dz)
            if abs(step) > abs(dz): step = dz
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], current_alt + step))
        except: self.takeoff_mode = False
    
    def gradual_landing(self):
        if not self.land_mode: return
        try:
            drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            current_alt = drone_pos[2]
            if current_alt <= 0.052:
                self.land_mode = self.is_airborne = False 
                self.sim.setObjectPosition(self.drone, self.drone, (0, 0, 0.05))
                print(f"\n[LANDING] Atterraggio completato.")
                return
            velocita_effettiva = self.land_speed
            if current_alt <= 0.25: velocita_effettiva = max(self.land_speed * (current_alt / 0.25), 0.04)
            step = velocita_effettiva * 0.05
            nuova_alt = max(current_alt - step, 0.05)
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], nuova_alt))
        except: self.land_mode = False

    def update_orbit(self):
        if not self.orbit_mode: return
        self.orbit_angle -= self.orbit_speed
        new_x = self.orbit_center[0] + self.orbit_radius * math.cos(self.orbit_angle)
        new_y = self.orbit_center[1] + self.orbit_radius * math.sin(self.orbit_angle)
        target_yaw = self.orbit_angle + math.pi
        self.sim.setObjectPosition(self.drone, self.sim.handle_world, (new_x, new_y, self.orbit_center[2]))
        self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, target_yaw))

    def process_aruco_detection(self, target_id):
        if not self.vision_sensor: return None, None
        try:
            img_buffer, resolution = self.sim.getVisionSensorImg(self.vision_sensor)
            if not img_buffer:
                return None, None
            
            res_x, res_y = int(resolution[0]), int(resolution[1])
            resolution_int = (res_x, res_y)
            
            img_array = np.frombuffer(img_buffer, dtype=np.uint8)
            num_channels = len(img_array) // (res_x * res_y)
            
            img_array.shape = (res_y, res_x, num_channels)
            img = cv2.flip(img_array, 0)
            
            if num_channels == 4:
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            else:
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                
            gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            gray_smoothed = cv2.GaussianBlur(gray, (3, 3), 0)
            
            corners, ids, _ = self.aruco_detector.detectMarkers(gray_smoothed)
            
            if ids is not None and len(ids) > 0:
                print(f" | [INFO VIDEO] Rilevati ID: {ids.flatten()}", end="", flush=True)
                for idx, marker_id in enumerate(ids.flatten()):
                    if marker_id == target_id:
                        marker_corner = corners[idx][0].reshape((4, 2))
                        return marker_corner, resolution_int
            else:
                print(" | [INFO VIDEO] Nessun marker intercettato", end="", flush=True)
                
        except Exception as e:
            print(f" | [CRASH DETECTOR]: {e}", end="", flush=True)
            
        return None, None

    def follow_target(self):
        try:
            # --- MODALITÀ ORIGINALE: TARGET DUMMY ---
            if self.waypoint_mode and self.target_dummy:
                pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                target_pos = self.sim.getObjectPosition(self.target_dummy, self.sim.handle_world)
                
                dist_gr_xy = math.sqrt((target_pos[0]-pos[0])**2 + (target_pos[1]-pos[1])**2)
                if dist_gr_xy < 1.5:
                    self.camera_fov = math.radians(20) 
                else:
                    self.camera_fov = math.radians(60) 
                self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)

                err_alt = target_pos[2] - pos[2]
                move_z = 0.0
                if abs(err_alt) > 0.5: move_z = math.copysign(0.04, err_alt)

                error_x = target_pos[0] - pos[0]
                error_y = target_pos[1] - pos[1]
                dist_totale = math.sqrt(error_x**2 + error_y**2)
                
                if dist_totale < self.ARRIVED_THRESH:
                    self.waypoint_mode = False
                    self.current_bearing_lock = None
                    print("\n[WAYPOINT] Target Dummy Raggiunto!")
                    return

                SOGLIA_ALLINEAMENTO = 0.05 
                target_global_yaw = 0.0
                is_active_lock = False

                if self.current_bearing_lock is None:
                    if abs(error_x) >= abs(error_y): self.current_bearing_lock = 'SINISTRA' if error_x > 0 else 'DESTRA'
                    else: self.current_bearing_lock = 'INDIETRO' if error_y > 0 else 'AVANTI'

                if self.current_bearing_lock == 'AVANTI':
                    if error_y < -SOGLIA_ALLINEAMENTO: target_global_yaw = math.radians(180.0); is_active_lock = True
                    else: self.current_bearing_lock = None
                elif self.current_bearing_lock == 'INDIETRO':
                    if error_y > SOGLIA_ALLINEAMENTO: target_global_yaw = math.radians(0.0); is_active_lock = True
                    else: self.current_bearing_lock = None
                elif self.current_bearing_lock == 'SINISTRA':
                    if error_x > SOGLIA_ALLINEAMENTO: target_global_yaw = math.radians(-90.0); is_active_lock = True
                    else: self.current_bearing_lock = None
                elif self.current_bearing_lock == 'DESTRA':
                    if error_x < -SOGLIA_ALLINEAMENTO: target_global_yaw = math.radians(90.0); is_active_lock = True
                    else: self.current_bearing_lock = None

                if is_active_lock:
                    ori_attuale = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
                    yaw_attuale = ori_attuale[2]
                    yaw_error = math.atan2(math.sin(target_global_yaw - yaw_attuale), math.cos(target_global_yaw - yaw_attuale))

                    if abs(yaw_error) > math.radians(8.0):
                        step_correzione = yaw_error * 0.4 
                        if abs(step_correzione) > self.STEP_YAW: step_correzione = math.copysign(self.STEP_YAW, step_correzione)
                        self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, step_correzione))
                    else:
                        self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, target_global_yaw))
                        self.sim.setObjectPosition(self.drone, self.drone, (0, self.STEP_MOVE, 0))

                if move_z != 0.0:
                    pos_attuale = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                    self.sim.setObjectPosition(self.drone, self.sim.handle_world, (pos_attuale[0], pos_attuale[1], pos_attuale[2] + move_z))

            # --- LOGICA DI MEMORIZZAZIONE POSIZIONE CON ARRESTO DI SICUREZZA ---
            elif self.aruco_mode:
                if self.current_path_index >= len(self.aruco_path):
                    self.aruco_mode = False
                    return

                target_marker_id = self.aruco_path[self.current_path_index]
                marker_corners, res = self.process_aruco_detection(target_marker_id)

                if marker_corners is not None and self.target_dummy:
                    # 1. Centro e dimensioni del marker in pixel
                    aruco_x = (marker_corners[0][0] + marker_corners[2][0]) / 2.0
                    aruco_y = (marker_corners[0][1] + marker_corners[2][1]) / 2.0
                    
                    pixel_error_x = aruco_x - (res[0] / 2.0)
                    pixel_error_y = (res[1] / 2.0) - aruco_y 

                    lunghezza_lato = math.sqrt((marker_corners[0][0] - marker_corners[1][0])**2 + (marker_corners[0][1] - marker_corners[1][1])**2)

                    # --- FRENO ELETTRONICO VISIVO ---
                    # Alza questo valore (es. 200.0) se si ferma troppo lontano, abbassalo (es. 130.0) se va a sbattere.
                    SOGLIA_DI_ARRESTO_PIXEL = 160.0

                    if lunghezza_lato < SOGLIA_DI_ARRESTO_PIXEL:
                        # FASE A: Il drone è ancora lontano. Calcola la posizione spaziale e memorizza
                        distanza_stimata = (120.0 / max(lunghezza_lato, 1.0)) * 2.0  
                        fov_orizzontale = self.camera_fov
                        metri_per_pixel = 2.0 * distanza_stimata * math.tan(fov_orizzontale / 2.0) / res[0]
                        
                        offset_locale_x = pixel_error_x * metri_per_pixel
                        offset_locale_z = pixel_error_y * metri_per_pixel  

                        drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                        raw_world_x = drone_pos[0] - offset_locale_x
                        raw_world_y = drone_pos[1] + distanza_stimata
                        raw_world_z = drone_pos[2] + offset_locale_z

                        # Casting float pulito per evitare l'errore di serializzazione CBOR/ZMQ
                        aruco_world_x = float(raw_world_x)
                        aruco_world_y = float(raw_world_y)
                        aruco_world_z = float(raw_world_z)

                        # Teletrasporta la sfera Target sulle coordinate calcolate dell'ArUco
                        self.sim.setObjectPosition(self.target_dummy, self.sim.handle_world, (aruco_world_x, aruco_world_y, aruco_world_z))
                        
                        # Forza lo switch immediato alla modalità waypoint nativa per farlo avanzare in questo ciclo
                        self.aruco_mode = False
                        self.waypoint_mode = True
                        self.current_bearing_lock = None
                    else:
                        # FASE B: Il drone ha raggiunto la distanza di sicurezza!
                        # Spegne sia la guida visiva sia i waypoint, congelando il drone in hovering sul posto
                        self.aruco_mode = False
                        self.waypoint_mode = False
                        self.current_bearing_lock = None
                        
                        # Aggiorna il dummy sulla posizione corrente per tenerlo fermo li
                        drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                        self.sim.setObjectPosition(self.target_dummy, self.sim.handle_world, drone_pos)
                        
                        print(f"\n[ARUCO] Alt! Distanza di sicurezza raggiunta. Lato marker: {lunghezza_lato:.1f}px. Hovering attivo.")

        except Exception as e:
            print(f"\n[ERRORE IN AVANZAMENTO]: {e}")

    def on_press(self, key): self.key_pressed = key   
    def on_release(self, key): self.key_pressed = None 

    def process_input(self):
        if not self.key_pressed: return 
        k = self.key_pressed
        try:
            if hasattr(k, 'char'):
                if k.char not in ['t', 'q'] and not self.is_airborne: return
                if k.char == 'w': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, self.STEP_MOVE))
                elif k.char == 's': 
                    drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                    if drone_pos[2] > 0.20: self.sim.setObjectPosition(self.drone, self.drone, (0, 0, -self.STEP_MOVE))
                elif k.char == 't': 
                    self.orbit_mode = self.waypoint_mode = self.aruco_mode = self.land_mode = False
                    self.takeoff_mode = True
                    self.key_pressed = None
                    time.sleep(0.2)
                elif k.char == 'g':
                    self.takeoff_mode = self.orbit_mode = self.waypoint_mode = self.aruco_mode = False
                    self.land_mode = True
                    self.key_pressed = None
                    time.sleep(0.2)
                elif k.char == 'a': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, self.STEP_YAW))
                elif k.char == 'd': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, -self.STEP_YAW))
                elif k.char == 'o':
                    self.takeoff_mode = self.land_mode = False
                    self.orbit_mode = not self.orbit_mode
                    if self.orbit_mode:
                        pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                        self.orbit_center = [pos[0], pos[1], pos[2]]
                        self.orbit_angle = 0
                    self.key_pressed = None
                    time.sleep(0.2)
                elif k.char == 'i': 
                    self.camera_tilt = min(self.camera_tilt + self.TILT_STEP, math.radians(60))
                    self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
                elif k.char == 'k': 
                    self.camera_tilt = max(self.camera_tilt - self.TILT_STEP, math.radians(-90))
                    self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
                
                # --- ZOOM MANUALE (u/j) ---
                elif k.char == 'u': 
                    if self.vision_sensor:
                        self.camera_fov = max(self.camera_fov - math.radians(2), math.radians(10))
                        self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
                elif k.char == 'j': 
                    if self.vision_sensor:
                        self.camera_fov = min(self.camera_fov + math.radians(2), math.radians(100))
                        self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
                
                # --- TASTI DI SWITCH DELLE MODALITÀ ---
                elif k.char == 'p':  
                    self.takeoff_mode = self.land_mode = self.aruco_mode = False
                    self.waypoint_mode = not self.waypoint_mode
                    self.current_bearing_lock = None 
                    print(f"\n[MODE] Navigazione Waypoint (Dummy): {'ON' if self.waypoint_mode else 'OFF'}")
                    self.key_pressed = None
                    time.sleep(0.2)
                    
                elif k.char == 'v':  
                    self.takeoff_mode = self.land_mode = self.waypoint_mode = False
                    self.aruco_mode = not self.aruco_mode
                    
                    if self.vision_sensor:
                        self.camera_fov = math.radians(60)
                        self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
                        
                    print(f"\n[MODE] Navigazione Visiva (ArUco): {'ON' if self.aruco_mode else 'OFF'}")
                    self.key_pressed = None
                    time.sleep(0.2)

                elif k.char == 'q': self.running = False
            
            if k == keyboard.Key.up: self.sim.setObjectPosition(self.drone, self.drone, (0, self.STEP_MOVE, 0))
            elif k == keyboard.Key.down: self.sim.setObjectPosition(self.drone, self.drone, (0, -self.STEP_MOVE, 0))
            elif k == keyboard.Key.left: self.sim.setObjectPosition(self.drone, self.drone, (-self.STEP_MOVE, 0, 0))
            elif k == keyboard.Key.right: self.sim.setObjectPosition(self.drone, self.drone, (self.STEP_MOVE, 0, 0))
        except: pass

    def run(self):
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        while self.running:
            self.process_input()
            if self.takeoff_mode: self.gradual_takeoff()
            elif self.land_mode: self.gradual_landing()
            if self.orbit_mode: self.update_orbit()
            
            if self.waypoint_mode or self.aruco_mode: 
                self.follow_target()
                
            line = self.get_telemetry()
            if line:
                if self.takeoff_mode: mode = "[TAKEOFF]"
                elif self.land_mode: mode = "[LANDING]"
                elif self.orbit_mode: mode = "[ORBITING]"
                elif self.waypoint_mode: mode = "[WAYPOINT_DUMMY]"
                elif self.aruco_mode: mode = "[FOLLOWING_ARUCO]"
                else: mode = "[MANUAL]"
                print(f"\r{line} {mode}", end="", flush=True)
            time.sleep(0.05) 
        listener.stop()

if __name__ == "__main__":
    controller = Drone_Controller()
    controller.run()