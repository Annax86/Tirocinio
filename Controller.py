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
            self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, math.pi))
            
            # Inizializzazione segnale camera (0 gradi)
            self.camera_tilt = 0.0
            self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
            
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
        
        # Parametri orbita
        self.orbit_mode = False
        self.orbit_center = [0, 0, 1]     # Centro dell'orbits
        self.orbit_radius = 0.5           # Raggio in metri
        self.orbit_angle = 0.0            # Angolo corrente
        self.orbit_speed = 0.05           # Velocità angolare 

        self.running = True
        self.waypoint_mode = False       
        self.key_pressed = None          

    def get_telemetry(self):
        """Calcola e restituisce i dati di volo e camera in tempo reale"""
        try:
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
            
            # Conversione GNSS
            lat = self.ref_lat + (pos[0] / self.LAT_METERS_PER_DEG)
            lon = self.ref_lon + (pos[1] / (self.LAT_METERS_PER_DEG * math.cos(math.radians(lat))))
            
            return (f"lat: {lat:.7f} | lon: {lon:.7f} | alt: {pos[2]:.2f}m | "
                    f"yaw: {math.degrees(ori[2]):.1f}° pitch: {math.degrees(ori[1]):.1f}° roll: {math.degrees(ori[0]):.1f}°")
        except:
            return None

    def update_orbit(self):
        """Calcola la posizione circolare (senso orario) e punta il drone verso il centro"""
        if not self.orbit_mode: return
        
        # 1. Incremento dell'angolo per il movimento circolare
        self.orbit_angle -= self.orbit_speed

        # 2. Calcolo coordinate (x, y) 
        new_x = self.orbit_center[0] + self.orbit_radius * math.cos(self.orbit_angle)
        new_y = self.orbit_center[1] + self.orbit_radius * math.sin(self.orbit_angle)

        # 3. Calcolo dello yaw per mantenere il drone rivolto verso il centro
        target_yaw = self.orbit_angle + math.pi

        # 4. Applicazione delle coordinate e rotazione
        self.sim.setObjectPosition(self.drone, self.sim.handle_world, (new_x, new_y, self.orbit_center[2]))
        self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, target_yaw))

    def follow_target(self):
        """Inseguimento del target"""
        if not self.target_dummy: return
        try: 
            target_pos = self.sim.getObjectPosition(self.target_dummy, self.sim.handle_world) 
            drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            
            dx, dy, dz = target_pos[0]-drone_pos[0], target_pos[1]-drone_pos[1], target_pos[2]-drone_pos[2]
            distanza = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distanza > 0.05:
                speed = math.sqrt(distanza)

                # Calcolo del passo di spostamento per il singolo frame (poiché il loop gira ogni 0.05 secondi)
                # Bisogna moltiplicare la velocità per il delta time per non far fare salti enormi al drone
                step = speed * 0.05

                # Se il passo calcolato supera la distanza rimanente, lo limitiano alla distanza
                if step > distanza:
                    step = distanza 

                self.sim.setObjectPosition(self.drone, self.sim.handle_world,
                                           (drone_pos[0] + (dx/distanza)*speed,
                                            drone_pos[1] + (dy/distanza)*speed,
                                            drone_pos[2] + (dz/distanza)*speed))
        except: pass

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
                # Controllo Quota (Z) (↑/↓)
                if k.char == 'w': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, self.STEP_MOVE))
                elif k.char == 's': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, -self.STEP_MOVE))
                
                # Rotazione (Yaw)
                elif k.char == 'a': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, self.STEP_YAW))
                elif k.char == 'd': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, -self.STEP_YAW))
                
                elif k.char == 'o':
                    self.orbit_mode = not self.orbit_mode
                    if self.orbit_mode:
                        # Imposta il centro dell'orbita 2 metri davanti alla posizione attuale
                        pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                        self.orbit_center = [pos[0], pos[1], pos[2]]
                        self.orbit_angle = 0
                        print(f"\n[ORBIT] ON - Centro: {self.orbit_center}")
                    time.sleep(0.2)

                # --- CONTROLLO TILT FOTOCAMERA ---
                elif k.char == 'i': # Ruota su
                    self.camera_tilt += self.TILT_STEP
                    if self.camera_tilt > math.radians(60): self.camera_tilt = math.radians(60)
                    self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
                elif k.char == 'k': # Ruota giù
                    self.camera_tilt -= self.TILT_STEP
                    if self.camera_tilt < math.radians(-90): self.camera_tilt = math.radians(-90)
                    self.sim.setFloatSignal("cameraTilt", self.camera_tilt)

                # Funzioni Rapide e Modalità
                elif k.char == 't': self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 1.0))
                elif k.char == 'g': self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05))
                elif k.char == 'p':
                    self.waypoint_mode = not self.waypoint_mode
                    print(f"\n[AUTO] {'ON' if self.waypoint_mode else 'OFF'}")
                    time.sleep(0.2)
                elif k.char == 'q': self.running = False
            
            # Frecce (Piano XY)
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

            if self.orbit_mode:
                self.update_orbit()

            elif self.waypoint_mode: 
                self.follow_target()

            line = self.get_telemetry()
            if line:
                if self.orbit_mode: mode = "[ORBITING]"
                elif self.waypoint_mode: mode = "[FOLLOWING]"
                else: mode = "[MANUAL]"
                print(f"\r{line} {mode}", end="", flush=True)
            
            time.sleep(0.05) 
            
        listener.stop()
        print("\nDisconnessione completata.")

if __name__ == "__main__":
    controller = Drone_Controller()
    controller.run()