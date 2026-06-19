import math
import time
import threading
import numpy as np
import cv2  # Richiede: pip install opencv-python opencv-contrib-python
import msvcrt  # Libreria nativa Windows per la gestione del buffer di input
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pynput import keyboard

class Drone_Controller:
    def __init__(self):
        """
        Inizializzazione del controllore globale per CoppeliaSim.
        Configura la connessione ZMQ, i sensori e le strutture dati asincrone.
        """
        try:
            # 1. CONNESSIONE ALLA REMOTE API DI COPPELIASIM (ZMQ)
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            
            # Recupero dell'handle del target fittizio associato al quadricottero
            self.drone = self.sim.getObject('/Quadcopter_target')
            
            # 2. CONFIGURAZIONE DEL SENSORE VISIVO (VISION SENSOR)
            try:
                self.vision_sensor = self.sim.getObject('/visionSensor')
                self.camera_fov = math.radians(60) 
                self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
                print("\033[1;32m[INFO] Vision Sensor inizializzato a 60°.\033[0m")
            except Exception as e:
                self.vision_sensor = None
                print(f"\033[1;31m[ERROR] Impossibile configurare /visionSensor: {e}\033[0m")
            
            # 3. PIPELINE ARUCO AGGIORNATA (Standard Ufficiali OpenCV 4.x)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

            # 4. RESET DELLO STATO INIZIALE DEL VELIVOLO NELLO SPAZIO 3D
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05))
            self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, 0.0))
            
            # Reset del segnale del servomotore Gimbal della telecamera (0 gradi = orizzontale)
            self.camera_tilt = 0.0
            self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
            
            # Disegna l'interfaccia iniziale pulita
            self.repaint_header()
        except Exception as e:
            print(f"\033[1;31mErrore critico durante l'inizializzazione: {e}\033[0m")
            exit()

        # --- PARAMETRI DI NAVIGAZIONE CINEMATICA E GEODETICA ---
        self.ref_lat, self.ref_lon = 0.0, 0.0
        self.LAT_METERS_PER_DEG = 111319.9  
        
        self.STEP_MOVE = 0.08              
        self.STEP_YAW = math.radians(5)    
        self.TILT_STEP = math.radians(2)   
        self.ARRIVED_THRESH = 0.10         # Soglia a 10cm per un aggancio ultra-preciso

        # Memoria condivisa thread-safe per il salvataggio dei Waypoint dell'autopilota
        self._lock = threading.Lock()
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = 0.8  

        # Registri per la localizzazione stimata tramite computer vision inversa
        self.aruco_pos_x = 0.0
        self.aruco_pos_y = 0.0
        self.has_aruco_pos = False
        self.aruco_pos_is_triangulated = False  # True = trilaterazione 2 marker, False = singolo marker (fallback)

        # Configurazione dinamica dello step di movimento frecce (impostata tramite 'm')
        self.movement_step_configured = False
        self.movement_pulse_ms = None
        self.movement_power = None

        # Flag di sincronizzazione visiva per evitare sovrapposizioni sul terminale
        self.gcs_input_active = False

        # --- VARIABILI DI STATO MACCHINA A STATI FINITI (FSM) ---
        self.takeoff_mode = False
        self.takeoff_target_alt = 0.8      
        self.takeoff_speed = 0.5           
        self.land_mode = False
        self.land_speed = 0.3              
        
        # Parametri orbita temporizzata richiesta da tastiera
        self.orbit_mode = False
        self.orbit_center = [0, 0, 1]     
        self.orbit_radius = 0.5           
        self.orbit_angle = 0.0            
        self.orbit_speed = 0.05           
        self.orbit_duration = None         
        self.orbit_start_time = 0.0        

        # Flag operativi delle modalità di guida autonome
        self.waypoint_mode = False         
        self.aruco_mode = False            
        self.localization_mode = False     
        self.is_searching_aruco = False

        # Sequenza programmata degli ID dei marker ArUco da scansionare nella serra
        self.aruco_path = [0, 1, 2, 3]             
        self.current_path_index = 0       

        self.running = True
        self.is_airborne = False           
        self.key_pressed = None          

        # --- TABELLA COLORI ESTETICI COMPATTI (ANTI LINE-WRAP) ---
        self.C_RESET = "\033[0m"
        self.C_GNSS  = "\033[1;37;42m GNSS \033[0m"       
        self.C_ARUCO = "\033[1;37;46m ARUCO \033[0m"       
        self.C_MANUAL = "\033[1;37;44m MANU \033[0m"     
        self.C_AUTO   = "\033[1;37;43m AUTO \033[0m"     
        self.C_ALERT  = "\033[1;37;41m ALER \033[0m"     

    def repaint_header(self):
        """Pulisce completamente il terminale e ristampa l'intestazione GCS fissa nelle prime 3 righe."""
        print("\033[H\033[J", end="") 
        print("\033[1;34m" + "="*62 + "\033[0m")
        print("\033[1;36m  DJI Mini 3 GCS: Monitoraggio e Controllo Real-Time \033[0m")
        print("\033[1;34m" + "="*62 + "\033[0m")

    # --- EQUAZIONE DI COMPENSAZIONE OTTICA PER LO ZOOM (TESI) ---
    def calculate_compensated_distance(self, lunghezza_lato):
        costante_calibrazione = 240.0 * math.tan(math.radians(30))
        tangente_fov_corrente = math.tan(self.camera_fov / 2.0)
        return costante_calibrazione / (max(lunghezza_lato, 1.0) * tangente_fov_corrente)

    # --- ENCAPSULAMENTO GETTER/SETTER THREAD-SAFE ---
    def get_target_coordinates(self):
        with self._lock:
            return self._target_x, self._target_y, self._target_z

    def set_target_coordinates(self, x, y, z):
        with self._lock:
            self._target_x = float(x)
            self._target_y = float(y)
            self._target_z = float(z)

    def get_telemetry_string(self):
        """Genera la riga di telemetria estetica super-compatta per evitare il line-wrap."""
        try:
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
            
            yaw_deg = math.degrees(ori[2])
            if abs(yaw_deg) < 0.05: yaw_deg = 0.0
            
            if self.localization_mode and self.has_aruco_pos:
                x_val, y_val = self.aruco_pos_x, self.aruco_pos_y
                badge_sensore = self.C_ARUCO if self.aruco_pos_is_triangulated else "\033[1;30;46m AR_1M \033[0m"
            else:
                x_val, y_val = pos[0], pos[1]
                badge_sensore = self.C_GNSS

            if self.takeoff_mode: badge_modo = self.C_ALERT + " TAKEOFF"
            elif self.land_mode: badge_modo = self.C_ALERT + " LANDING"
            elif self.orbit_mode: badge_modo = self.C_AUTO + " ORBIT"
            elif self.waypoint_mode: badge_modo = self.C_AUTO + " WP_NAV"
            elif self.aruco_mode: badge_modo = self.C_AUTO + " AR_SCAN"
            elif self.localization_mode: badge_modo = self.C_ARUCO + " LOC_ON"
            else: badge_modo = self.C_MANUAL + " HOVER"

            if self.movement_step_configured:
                step_info = f"STEP:{self.STEP_MOVE:.3f}m"
            else:
                step_info = "\033[1;33mSTEP:?\033[0m"

            str_telemetria = (
                f"{badge_sensore} │ X:{x_val:5.2f} │ Y:{y_val:5.2f} │ Z:{pos[2]:4.2f} │ "
                f"Ψ:{yaw_deg:5.1f}° │ {badge_modo} │ {step_info}"
            )
            return str_telemetria
        except:
            return None

    # --- OPERAZIONI ASINCRONE DELLA GROUND CONTROL STATION (THREAD) ---
    def ask_coordinates_thread(self):
        self.key_pressed = None
        self.gcs_input_active = True  
        print("\n\033[1;36m" + "="*55)
        print(" [GCS COMMAND] INSERIMENTO TARGET EMBEDDED")
        print("="*55 + "\033[0m")
        try:
            x = float(input(" -> Coordinata Target X (metri): "))
            y = float(input(" -> Coordinata Target Y (metri): "))
            z = float(input(" -> Quota di Volo Target Z (metri): "))
            self.set_target_coordinates(x, y, z)
            self.waypoint_mode = True
            print(f"\033[1;32m [✓] Rotta calcolata. Autopilota agganciato.\033[0m")
        except ValueError:
            print("\033[1;31m [X] Input non numerico. Procedura abortita.\033[0m")
            self.waypoint_mode = False
        
        time.sleep(1.5) 
        self.repaint_header() 
        self.key_pressed = None
        self.gcs_input_active = False 

    def ask_movement_command_thread(self):
        """
        Chiede all'utente i parametri (durata impulso e potenza) usati per calcolare
        lo STEP_MOVE applicato a ogni pressione delle frecce direzionali.
        Non muove il drone: si limita a configurare il passo di spostamento.
        """
        self.key_pressed = None
        self.gcs_input_active = True
        print("\n\033[1;36m" + "="*55)
        print(" [GCS COMMAND] CONFIGURAZIONE PASSO DI MOVIMENTO FRECCE")
        print("="*55 + "\033[0m")
        try:
            tempo = float(input(" -> Durata impulso elettrico (millisecondi): "))
            potenza = float(input(" -> Coefficiente Potenza applicata (es. 0.05): "))

            tempo_s = tempo / 1000.0
            spazio = tempo_s * potenza * 10.0

            self.movement_pulse_ms = tempo
            self.movement_power = potenza
            self.STEP_MOVE = spazio
            self.movement_step_configured = True

            print(f"\033[1;32m [✓] Passo di movimento impostato: {spazio:.4f} metri per pressione freccia.\033[0m")
        except ValueError:
            print("\033[1;31m [X] Dati immessi errati. Configurazione non modificata.\033[0m")

        time.sleep(1.5)
        self.repaint_header()
        self.key_pressed = None
        self.gcs_input_active = False

    def ask_orbit_command_thread(self):
        self.key_pressed = None
        self.gcs_input_active = True
        print("\n\033[1;36m" + "="*55)
        print(" [GCS COMMAND] CONFIGURAZIONE TRAIETTORIA CIRCOLARE")
        print("="*55 + "\033[0m")
        try:
            durata = float(input(" -> Durata temporizzatore orbita (secondi): "))
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            
            self.orbit_center = [pos[0], pos[1], pos[2]]
            self.orbit_angle = 0.0
            self.orbit_duration = durata
            self.orbit_start_time = time.time() 
            self.orbit_mode = True
            print(f"\033[1;32m [✓] Orbita agganciata per {durata}s.\033[0m")
        except ValueError:
            print("\033[1;31m [X] Valore temporale errato.\033[0m")
            self.orbit_mode = False
        
        time.sleep(1.5)
        self.repaint_header()
        self.key_pressed = None
        self.gcs_input_active = False

    # --- FUNZIONI DI DINAMICA INTERNA ED FISICA DI VOLO ---
    def gradual_takeoff(self):
        if not self.takeoff_mode: return
        try:
            drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            current_alt = drone_pos[2]
            dz = self.takeoff_target_alt - current_alt
            
            if abs(dz) < 0.05: 
                self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], self.takeoff_target_alt))
                self.takeoff_mode = False
                self.is_airborne = True  
                print(f"\n\033[1;32m[SYSTEM] Decollo ultimato. Velivolo stabilizzato a {self.takeoff_target_alt}m.\033[0m")
                return
                
            step = self.takeoff_speed * 0.05 if dz > 0 else -self.takeoff_speed * 0.05
            if abs(step) > abs(dz): step = dz
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], current_alt + step))
        except: 
            self.takeoff_mode = False
    
    def gradual_landing(self):
        if not self.land_mode: return
        try:
            drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            current_alt = drone_pos[2]
            if current_alt <= 0.052:
                self.land_mode = self.is_airborne = False 
                self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], 0.05))
                print(f"\n\033[1;31m[SYSTEM] Touchdown rilevato. Motori spenti.\033[0m")
                return
            velocita_effettiva = self.land_speed
            if current_alt <= 0.25:
                velocita_effettiva = max(self.land_speed * (current_alt / 0.25), 0.04)
            step = velocita_effettiva * 0.05
            nuova_alt = max(current_alt - step, 0.05)
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], nuova_alt))
        except: self.land_mode = False

    def update_orbit(self):
        if not self.orbit_mode: return
        if self.orbit_duration is not None:
            if time.time() - self.orbit_start_time >= self.orbit_duration:
                self.orbit_mode = False
                print("\n\033[1;33m[SYSTEM] Timer orbita scaduto.\033[0m")
                return
                
        self.orbit_angle -= self.orbit_speed
        new_x = self.orbit_center[0] + self.orbit_radius * math.cos(self.orbit_angle)
        new_y = self.orbit_center[1] + self.orbit_radius * math.sin(self.orbit_angle)
        target_yaw = self.orbit_angle + math.pi
        self.sim.setObjectPosition(self.drone, self.sim.handle_world, (new_x, new_y, self.orbit_center[2]))
        self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, target_yaw))

    # --- SOTTO-SISTEMI DI VISIONE COMPUTAZIONALE OPENCV ---
    def process_aruco_detection(self, target_id):
        if not self.vision_sensor: return None, None
        try:
            img_buffer, resolution = self.sim.getVisionSensorImg(self.vision_sensor)
            if not img_buffer or len(img_buffer) == 0: return None, None
            
            img = np.frombuffer(img_buffer, dtype=np.uint8)
            img.shape = (resolution[1], resolution[0], 3)
            img = cv2.flip(img, 0)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            if ids is not None:
                for idx, marker_id in enumerate(ids.flatten()):
                    if marker_id == target_id:
                        return corners[idx][0], resolution
        except: pass
        return None, None

    def process_all_aruco_detections(self):
        """
        Rileva TUTTI i marker ArUco visibili in un singolo frame.
        Necessario per la trilaterazione: servono almeno due marker
        contemporaneamente nel campo visivo della telecamera.
        Ritorna una lista di tuple (marker_id, corners_4x2).
        """
        if not self.vision_sensor: return []
        try:
            img_buffer, resolution = self.sim.getVisionSensorImg(self.vision_sensor)
            if not img_buffer or len(img_buffer) == 0: return []

            img = np.frombuffer(img_buffer, dtype=np.uint8)
            img.shape = (resolution[1], resolution[0], 3)
            img = cv2.flip(img, 0)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            risultati = []
            if ids is not None:
                for idx, marker_id in enumerate(ids.flatten()):
                    risultati.append((int(marker_id), corners[idx][0]))
            return risultati
        except:
            return []

    def get_marker_world_position(self, marker_id):
        """Recupera la posizione assoluta nota (beacon) di un marker dalla scena."""
        nomi_scena_da_testare = [
            f'/Aruco_{marker_id}', f'@Aruco_{marker_id}', f'Aruco_{marker_id}',
            f'/aruco_marker_{marker_id}', f'@aruco_marker_{marker_id}'
        ]
        for nome in nomi_scena_da_testare:
            try:
                marker_handle = self.sim.getObject(nome)
                if marker_handle is not None:
                    return self.sim.getObjectPosition(marker_handle, self.sim.handle_world)
            except:
                continue
        return None

    def trilateration_2d(self, x1, y1, r1, x2, y2, r2, fallback_xy):
        """
        Trilaterazione 2D classica: dati due beacon a posizione nota (x1,y1) e (x2,y2)
        e le rispettive distanze stimate (r1, r2) dal punto da localizzare, calcola
        l'intersezione dei due cerchi. Questo è il metodo realmente impiegato nei
        sistemi di localizzazione senza GPS (es. beacon UWB/ottici a posizione nota).

        Tra le due intersezioni possibili si scarta quella più lontana dalla stima
        di fallback (es. l'ultima posizione conosciuta del drone), per risolvere
        l'ambiguità geometrica.
        """
        d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if d < 1e-6:
            return None  # beacon coincidenti, geometria degenere

        # Se i cerchi non si intersecano (rumore di stima), si effettua un clamp
        # morbido sui raggi per ottenere comunque una soluzione approssimata.
        if d > (r1 + r2):
            eccesso = d - (r1 + r2)
            r1 += eccesso / 2.0
            r2 += eccesso / 2.0
        elif d < abs(r1 - r2):
            if r1 > r2: r1 = d + r2 - 1e-6
            else: r2 = d + r1 - 1e-6

        a = (r1**2 - r2**2 + d**2) / (2 * d)
        h_sq = r1**2 - a**2
        h = math.sqrt(max(h_sq, 0.0))

        xm = x1 + a * (x2 - x1) / d
        ym = y1 + a * (y2 - y1) / d

        # Le due possibili soluzioni (perpendicolare alla retta tra i beacon)
        sol1_x = xm + h * (y2 - y1) / d
        sol1_y = ym - h * (x2 - x1) / d
        sol2_x = xm - h * (y2 - y1) / d
        sol2_y = ym + h * (x2 - x1) / d

        fx, fy = fallback_xy
        dist1 = (sol1_x - fx)**2 + (sol1_y - fy)**2
        dist2 = (sol2_x - fx)**2 + (sol2_y - fy)**2

        return (sol1_x, sol1_y) if dist1 <= dist2 else (sol2_x, sol2_y)

    # --- CORE ENGINE DI NAVIGAZIONE AUTONOMA ---
    def follow_target(self):
        try:
            if self.localization_mode:
                rilevamenti = self.process_all_aruco_detections()

                # Filtra solo i marker con lato sufficientemente grande da essere
                # considerati attendibili per la stima di distanza ottica.
                beacon_validi = []
                for marker_id, corners in rilevamenti:
                    lunghezza_lato = math.sqrt((corners[0][0] - corners[1][0])**2 + (corners[0][1] - corners[1][1])**2)
                    if lunghezza_lato < 15.0:
                        continue
                    marker_pos_world = self.get_marker_world_position(marker_id)
                    if marker_pos_world is None:
                        continue
                    distanza_stimata = self.calculate_compensated_distance(lunghezza_lato)
                    beacon_validi.append((marker_id, marker_pos_world[0], marker_pos_world[1], distanza_stimata))

                if len(beacon_validi) >= 2:
                    # TRIANGOLAZIONE VERA: due beacon a posizione nota + due distanze
                    # stimate otticamente -> intersezione dei due cerchi (trilaterazione 2D).
                    id1, x1, y1, r1 = beacon_validi[0]
                    id2, x2, y2, r2 = beacon_validi[1]

                    pos_attuale = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                    fallback_xy = (pos_attuale[0], pos_attuale[1])

                    soluzione = self.trilateration_2d(x1, y1, r1, x2, y2, r2, fallback_xy)
                    if soluzione is not None:
                        self.aruco_pos_x, self.aruco_pos_y = soluzione
                        self.has_aruco_pos = True
                        self.aruco_pos_is_triangulated = True
                    else:
                        self.has_aruco_pos = False

                elif len(beacon_validi) == 1:
                    # Solo un beacon visibile: fallback alla stima a singolo marker
                    # (meno precisa, usata solo come ripiego momentaneo).
                    marker_id, mx, my, distanza_stimata = beacon_validi[0]
                    drone_ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
                    angolo_vista_reale = drone_ori[2] + (math.pi / 2)

                    self.aruco_pos_x = mx - (distanza_stimata * math.cos(angolo_vista_reale))
                    self.aruco_pos_y = my - (distanza_stimata * math.sin(angolo_vista_reale))
                    self.has_aruco_pos = True
                    self.aruco_pos_is_triangulated = False
                else:
                    self.has_aruco_pos = False
                return

            if self.waypoint_mode:
                pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
                
                tgt_x, tgt_y, tgt_z = self.get_target_coordinates()
                err_alt = tgt_z - pos[2]
                
                if abs(err_alt) > 0.5: 
                    move_z = 0.04 if err_alt > 0 else -0.04
                    self.sim.setObjectPosition(self.drone, self.sim.handle_world, (pos[0], pos[1], pos[2] + move_z))
                    return  
                
                x_corrente = self.aruco_pos_x if self.has_aruco_pos else pos[0]
                y_corrente = self.aruco_pos_y if self.has_aruco_pos else pos[1]

                cur_lat = self.ref_lat + (x_corrente / self.LAT_METERS_PER_DEG) 
                cur_lon = self.ref_lon + (y_corrente / (self.LAT_METERS_PER_DEG * math.cos(math.radians(cur_lat)))) 
                end_lat = self.ref_lat + (tgt_x / self.LAT_METERS_PER_DEG) 
                end_lon = self.ref_lon + (tgt_y / (self.LAT_METERS_PER_DEG * math.cos(math.radians(end_lat)))) 
                
                R = 6371000.0  
                dphi = math.radians(end_lat - cur_lat) 
                dlam = math.radians(end_lon - cur_lon) 
                phi1, phi2 = math.radians(cur_lat), math.radians(end_lat) 
                
                a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2)**2 
                dist = R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)) 
                
                # ARRESTO DI PRECISIONE E HARD LOCK POSIZIONALE
                if dist < self.ARRIVED_THRESH:
                    self.sim.setObjectPosition(self.drone, self.sim.handle_world, (tgt_x, tgt_y, tgt_z))
                    self.waypoint_mode = False 
                    self.key_pressed = None  
                    
                    if self.is_searching_aruco:
                        print(f"\n\033[1;32m[MISSION] Pianta {self.aruco_path[self.current_path_index]} Raggiunta! Avvio campionamento.\033[0m")
                        if self.current_path_index < len(self.aruco_path) - 1:
                            self.current_path_index += 1
                        self.is_searching_aruco = False
                    else:
                        print(f"\n\033[1;32m[GCS] Navigazione completata.\033[0m")
                    return

                x = math.sin(dlam) * math.cos(phi2) 
                y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam) 
                target_bearing = (math.degrees(math.atan2(x, y)) + 360) % 360 
                
                cur_yaw_deg = (math.degrees(ori[2]) + 90 + 360) % 360 
                err_ang = target_bearing - cur_yaw_deg 
                if err_ang > 180: err_ang -= 360 
                elif err_ang < -180: err_ang += 360 

                ANGLE_TOLERANCE = 5.0  
                aligned = False
                
                if abs(err_ang) <= ANGLE_TOLERANCE: 
                    aligned = True 
                else:
                    step_yaw = math.degrees(self.STEP_YAW) if err_ang > 0 else -math.degrees(self.STEP_YAW)
                    if abs(step_yaw) > abs(err_ang): step_yaw = err_ang 
                    nuovo_yaw = math.radians(math.degrees(ori[2]) + step_yaw) 
                    self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (ori[0], ori[1], nuovo_yaw)) 

                move_x, move_y = 0.0, 0.0 
                if aligned: 
                    if dist > 0.50:
                        speed_modifier = self.STEP_MOVE
                    else:
                        speed_modifier = self.STEP_MOVE * (dist / 0.50)
                        speed_modifier = max(speed_modifier, 0.005)  
                        
                    # GUIDA RETTILINEA DI PRECISIONE PURA DIREZIONE IPOTENUSA TARGET
                    angolo_verso_target = math.atan2(tgt_y - pos[1], tgt_x - pos[0])
                    move_x = speed_modifier * math.cos(angolo_verso_target)
                    move_y = speed_modifier * math.sin(angolo_verso_target)
                
                self.sim.setObjectPosition(self.drone, self.sim.handle_world, (pos[0] + move_x, pos[1] + move_y, pos[2]))

            elif self.aruco_mode:
                # Aggancia QUALSIASI marker ArUco visibile nel campo della telecamera,
                # indipendentemente dal suo ID. Tra più marker visibili, si privilegia
                # quello con il lato maggiore (il più vicino/grande nell'immagine).
                rilevamenti = self.process_all_aruco_detections()
                if not rilevamenti:
                    # Nessun marker visibile: nessun avanzamento alla cieca,
                    # si resta fermi finché non ne viene agganciato uno.
                    return

                target_marker_id, marker_corners = max(
                    rilevamenti,
                    key=lambda r: math.sqrt((r[1][0][0] - r[1][1][0])**2 + (r[1][0][1] - r[1][1][1])**2)
                )

                lunghezza_lato = math.sqrt((marker_corners[0][0] - marker_corners[1][0])**2 + (marker_corners[0][1] - marker_corners[1][1])**2)
                if lunghezza_lato < 15.0: return 

                distanza_totale_stimata = self.calculate_compensated_distance(lunghezza_lato)
                
                CUSCINO_SICUREZZA = 1.45  
                distanza_navigazione = distanza_totale_stimata - CUSCINO_SICUREZZA

                drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                drone_ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
                angolo_vista_reale = drone_ori[2] + (math.pi / 2)
                quota_vincolata_tavolo = 0.8

                # ARRESTO FINALE: il drone è ormai al cuscinetto di sicurezza dal marker.
                # Si blocca la posizione corrente, niente più avvicinamento.
                SOGLIA_ARRESTO_DISTANZA = 0.10
                if distanza_navigazione <= SOGLIA_ARRESTO_DISTANZA:
                    self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], quota_vincolata_tavolo))
                    self.aruco_mode = False
                    self.waypoint_mode = False
                    self.is_searching_aruco = False
                    print(f"\n\033[1;32m[VISION] Marker ID {target_marker_id} raggiunto. Arresto a {CUSCINO_SICUREZZA}m dal marker.\033[0m")
                    return

                # AGGIORNAMENTO CONTINUO: il target viene ricalcolato ad ogni frame
                # in base alla posizione corrente del marker rilevato dalla telecamera,
                # così il drone si autocorregge mentre si avvicina.
                distanza_navigazione = max(distanza_navigazione, 0.0)
                world_x = drone_pos[0] + (distanza_navigazione * math.cos(angolo_vista_reale))
                world_y = drone_pos[1] + (distanza_navigazione * math.sin(angolo_vista_reale))

                self.set_target_coordinates(world_x, world_y, quota_vincolata_tavolo)

                # Avanzamento graduale verso il target ricalcolato, restando in aruco_mode
                # in modo da continuare a rilevare il marker fotogramma per fotogramma.
                dist_residua = math.sqrt((world_x - drone_pos[0])**2 + (world_y - drone_pos[1])**2)
                if dist_residua > 0.01:
                    speed_modifier = min(self.STEP_MOVE, dist_residua)
                    move_x = speed_modifier * math.cos(angolo_vista_reale)
                    move_y = speed_modifier * math.sin(angolo_vista_reale)
                    self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0] + move_x, drone_pos[1] + move_y, quota_vincolata_tavolo))
        except:
            pass

    # --- METODI DI INTERFACCIA TASTIERA DI CLASSE ---
    def on_press(self, key): 
        self.key_pressed = key   

    def on_release(self, key): 
        self.key_pressed = None 

    def process_input(self):
        if not self.key_pressed: return 
        if self.gcs_input_active:
            self.key_pressed = None
            return
            
        k = self.key_pressed
        try:
            if hasattr(k, 'char') and k.char is not None:
                char = k.char.lower()
                if char not in ['t', 'q'] and not self.is_airborne: return
                
                if char == 'w':
                    self.takeoff_mode = self.land_mode = False 
                    self.sim.setObjectPosition(self.drone, self.drone, (0, 0, self.STEP_MOVE))
                elif char == 's': 
                    self.takeoff_mode = self.land_mode = False 
                    drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                    if drone_pos[2] > 0.20:
                        nuva_alt_manuale = drone_pos[2] - self.STEP_MOVE
                        if nuva_alt_manuale < 0.20:
                            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], 0.20))
                        else:
                            self.sim.setObjectPosition(self.drone, self.drone, (0, 0, -self.STEP_MOVE))
                elif char == 't': 
                    self.orbit_mode = self.waypoint_mode = self.land_mode = self.localization_mode = False
                    self.takeoff_mode = True
                    self.key_pressed = None
                    time.sleep(0.2)
                elif char == 'g':
                    self.takeoff_mode = self.orbit_mode = self.waypoint_mode = self.localization_mode = False
                    self.land_mode = True
                    self.key_pressed = None
                    time.sleep(0.2)
                elif char == 'a': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, self.STEP_YAW))
                elif char == 'd': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, -self.STEP_YAW))
                elif char == 'o':
                    self.takeoff_mode = self.land_mode = self.waypoint_mode = self.aruco_mode = self.localization_mode = False
                    self.key_pressed = None
                    threading.Thread(target=self.ask_orbit_command_thread, daemon=True).start()
                    time.sleep(0.2)
                elif char == 'i': 
                    self.camera_tilt += self.TILT_STEP
                    if self.camera_tilt > math.radians(50): self.camera_tilt = math.radians(50) 
                    self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
                elif char == 'k': 
                    self.camera_tilt -= self.TILT_STEP
                    if self.camera_tilt < math.radians(-90): self.camera_tilt = math.radians(-90)
                    self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
                elif char == 'u': 
                    self.camera_fov -= math.radians(2)
                    if self.camera_fov < math.radians(10): self.camera_fov = math.radians(10)
                    self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
                elif char == 'j': 
                    self.camera_fov += math.radians(2)
                    if self.camera_fov > math.radians(100): self.camera_fov = math.radians(100)
                    self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
                elif char == 'p':  
                    self.takeoff_mode = self.land_mode = self.aruco_mode = self.localization_mode = False
                    self.is_searching_aruco = False 
                    self.key_pressed = None
                    threading.Thread(target=self.ask_coordinates_thread, daemon=True).start()
                    time.sleep(0.2)
                elif char == 'm':  
                    self.key_pressed = None
                    threading.Thread(target=self.ask_movement_command_thread, daemon=True).start()
                    time.sleep(0.2)
                elif char == 'v':  
                    self.takeoff_mode = self.land_mode = self.waypoint_mode = self.localization_mode = False
                    self.aruco_mode = not self.aruco_mode
                    self.key_pressed = None
                    time.sleep(0.2)
                elif char == 'l':  
                    self.takeoff_mode = self.land_mode = self.waypoint_mode = self.aruco_mode = False
                    self.localization_mode = not self.localization_mode
                    if not self.localization_mode:
                        self.has_aruco_pos = False
                        self.aruco_pos_is_triangulated = False
                    self.key_pressed = None
                    time.sleep(0.2)
                elif char == 'q': self.running = False
            
            else:
                if not self.is_airborne: return
                if k in (keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right):
                    if not self.movement_step_configured:
                        self.key_pressed = None
                        threading.Thread(target=self.ask_movement_command_thread, daemon=True).start()
                        time.sleep(0.2)
                        return

                if k == keyboard.Key.up: self.sim.setObjectPosition(self.drone, self.drone, (0, self.STEP_MOVE, 0)) 
                elif k == keyboard.Key.down: self.sim.setObjectPosition(self.drone, self.drone, (0, -self.STEP_MOVE, 0)) 
                elif k == keyboard.Key.left: self.sim.setObjectPosition(self.drone, self.drone, (-self.STEP_MOVE, 0, 0)) 
                elif k == keyboard.Key.right: self.sim.setObjectPosition(self.drone, self.drone, (self.STEP_MOVE, 0, 0)) 
        except: pass

    def run(self):
        """Avvia il loop periodico a 20Hz ancorando stabilmente la riga di testo."""
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        while self.running:
            self.process_input()
            
            # Svuota istantaneamente il buffer hardware nascosto di Windows ad ogni loop
            while msvcrt.kbhit():
                msvcrt.getch()
                
            if self.takeoff_mode: self.gradual_takeoff()
            elif self.land_mode: self.gradual_landing()
            if self.orbit_mode: self.update_orbit()
            elif self.waypoint_mode or self.aruco_mode or self.localization_mode: self.follow_target()
                
            if not self.gcs_input_active:
                line = self.get_telemetry_string()
                if line:
                    # \033[4;1H blocca rigidamente il testo alla riga 4 dello schermo senza mai duplicarlo
                    print(f"\033[4;1H{line}\033[K", end="", flush=True) 
            time.sleep(0.05) 
        listener.stop()

if __name__ == "__main__":
    controller = Drone_Controller()
    controller.run()