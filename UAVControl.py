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
            # Ricerca dell'oggetto Dummy nella scena. 
            try:
                self.target_dummy = self.sim.getObject('/Target')
                print("[INFO] Target Dummy trovato. Navigazione generica pronta.")
            except:
                self.target_dummy = None
                print("[WARNING] Dummy 'Target' non trovato. Potrai usare solo i comandi manuali.")

            # 3. RESET POSIZIONE (Cinematica)
            # Posizionamento del drone al centro (0,0) e a 5cm da terra per evitare collisioni all'avvio
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05))
            self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, 0))
            
            print("--- DJI Mini 3: Sistema di Navigazione GNSS Pronto ---")
        except Exception as e:
            print(f"Errore inizializzazione: {e}")
            exit()

        # --- PARAMETRI DI NAVIGAZIONE E GEODETICA ---
        self.ref_lat, self.ref_lon = 0.0, 0.0  # Punto di origine geografico (0,0)
        self.LAT_METERS_PER_DEG = 111319.9     # Metri in un grado di latitudine (WGS84)
        
        # Parametri di volo (Velocità < 10m/s)
        self.STEP_MOVE = 0.08            # Spostamento in metri per ogni pressione tasto
        self.STEP_YAW = math.radians(5)  # Rotazione di 5 gradi (convertiti in radianti)
        self.running = True
        self.waypoint_mode = False       # Insegue il target
        self.key_pressed = None          # Gestione tasto singolo per evitare sovraccarico

    def get_telemetry(self):
        """Calcola e restituisce i dati di volo in tempo reale"""
        try:
            # Recupero coordinate cartesiane (X, Y, Z) dal simulatore
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            
            # Recupero orientamento (Roll, Pitch, Yaw)
            ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
            
            # CONVERSIONE IN COORDINATE GNSS (GPS/GLONASS/Galileo)
            # Calcolo Latitudine basato sull'asse X
            lat = self.ref_lat + (pos[0] / self.LAT_METERS_PER_DEG)

            # Calcolo Longitudine (corretta per la curvatura terrestre in base alla Lat)
            lon = self.ref_lon + (pos[1] / (self.LAT_METERS_PER_DEG * math.cos(math.radians(lat))))
            
            # Formattazione stringa di telemetria 
            return (f"Lat: {lat:.7f} | Lon: {lon:.7f} | Alt: {pos[2]:.2f}m | "
                    f"Y: {math.degrees(ori[2]):.1f}° P: {math.degrees(ori[1]):.1f}° R: {math.degrees(ori[0]):.1f}°")
        except:
            return None

    def follow_target(self):
        """Inseguimento del target"""

        if not self.target_dummy: return
        try: 
            # 1. Legge la posizione attuale del Dummy
            target_pos = self.sim.getObjectPosition(self.target_dummy, self.sim.handle_world) 
            
            # 2. Legge la posizione attuale del drone 
            drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            
            # 3. Calcolo del vettoriale del distacco 
            dx, dy, dz = target_pos[0]-drone_pos[0], target_pos[1]-drone_pos[1], target_pos[2]-drone_pos[2]
            distanza = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Se il drone è lontano più di 10 cm dal Dummy, si muove
            if distanza > 0.15:
                speed = 0.05 # Velocità di inseguimento
                self.sim.setObjectPosition(self.drone, self.sim.handle_world,
                                           (drone_pos[0] + (dx/distanza)*speed,
                                            drone_pos[1] + (dy/distanza)*speed,
                                            drone_pos[2] + (dz/distanza)*speed))
        except: pass

    def on_press(self, key): 
        self.key_pressed = key  # Memorizza solo l'ultimo tasto premuto 

    def on_release(self, key):
        self.key_pressed = None # Pulisce al rilascio

    def process_input(self):
        """Gestione dei comandi uno alla volta nel ciclo principale"""

        if not self.key_pressed: return 

        k = self.key_pressed
        try:
            # COMANDI ALFANUMERICI (W, S, A, D, T, G, Q, P)
            if hasattr(k, 'char'):
                # Controllo Quota (Z)
                if k.char == 'w': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, self.STEP_MOVE)) # Sali
                elif k.char == 's': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, -self.STEP_MOVE)) # Scendi
                
                # Rotazione (Yaw)
                elif k.char == 'a': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, self.STEP_YAW)) # Sinistra
                elif k.char == 'd': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, -self.STEP_YAW)) # Destra
                
                # Funzioni Rapide
                elif k.char == 't': self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 1.0)) # Decollo auto
                elif k.char == 'g': self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05)) # Atterraggio auto
                
                # Modalità waypoint
                elif k.char == 'p':
                    self.waypoint_mode = not self.waypoint_mode
                    print(f"\n[AUTO] {'ON' if self.waypoint_mode else 'OFF'}")
                    time.sleep(0.2) # Debounce per evitare toggle infiniti

                # Chiusura
                elif k.char == 'q': self.running = False
            
            # COMANDI DIREZIONALI (Frecce - Piano XY)
            if k == keyboard.Key.up: self.sim.setObjectPosition(self.drone, self.drone, (self.STEP_MOVE, 0, 0))    # Avanti
            elif k == keyboard.Key.down: self.sim.setObjectPosition(self.drone, self.drone, (-self.STEP_MOVE, 0, 0)) # Indietro
            elif k == keyboard.Key.left: self.sim.setObjectPosition(self.drone, self.drone, (0, self.STEP_MOVE, 0))  # Sinistra
            elif k == keyboard.Key.right: self.sim.setObjectPosition(self.drone, self.drone, (0, -self.STEP_MOVE, 0)) # Destra
        except:
            pass # Ignora errori di pressione tasti non mappati

    def run(self):
        """Ciclo principale di esecuzione"""
        # Listener configurato per aggiornare solo lo stato dei tasti
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        
        while self.running:
            # 1. Gestione Input (sequenziale)
            self.process_input()

            # 2. Gestione Automatica
            if self.waypoint_mode:
                self.follow_target()

            # 3. Telemetria
            line = self.get_telemetry()
            if line:
                mode = "[FOLLOWING]" if self.waypoint_mode else "[MANUAL]"
                # Stampa sulla stessa riga (\r) 
                print(f"\r{line} {mode}", end="", flush=True)
            
            # ATTESA DI SICUREZZA (0.1s = 10Hz)
            # Per non saturare la connessione ZMQ e prevenire i crash
            time.sleep(0.05) 
            
        listener.stop()
        print("\nDisconnessione completata.")

if __name__ == "__main__":
    controller = Drone_Controller()
    controller.run()