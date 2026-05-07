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
            
            # 2. RESET POSIZIONE (Cinematica)
            # Portiamo il drone al centro (0,0) a 5cm da terra per evitare collisioni all'avvio
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05))
            self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, 0))
            
            print("--- DJI Mini 3: Sistema di Navigazione GNSS Pronto ---")
        except Exception as e:
            print(f"Errore inizializzazione: {e}")
            exit()

        # --- PARAMETRI DI NAVIGAZIONE E GEODETICA ---
        self.ref_lat, self.ref_lon = 0.0, 0.0  # Punto di origine geografico (0,0)
        self.LAT_METERS_PER_DEG = 111319.9     # Metri in un grado di latitudine (WGS84)
        
        # Parametri di volo (Velocità < 10m/s come da specifiche DJI)
        self.STEP_MOVE = 0.08  # Spostamento in metri per ogni pressione tasto
        self.STEP_YAW = math.radians(5)  # Rotazione di 5 gradi (convertiti in radianti)
        self.running = True

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
            
            # Formattazione stringa di telemetria (Leggera e leggibile)
            return (f"Lat: {lat:.7f} | Lon: {lon:.7f} | Alt: {pos[2]:.2f}m | "
                    f"Y: {math.degrees(ori[2]):.1f}° P: {math.degrees(ori[1]):.1f}° R: {math.degrees(ori[0]):.1f}°")
        except:
            return None

    def on_press(self, key):
        """Gestore degli input da tastiera"""
        try:
            # COMANDI ALFANUMERICI (W, S, A, D, T, G, Q)
            if hasattr(key, 'char'):
                # Controllo Quota (Z)
                if key.char == 'w': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, self.STEP_MOVE)) # Sali
                elif key.char == 's': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, -self.STEP_MOVE)) # Scendi
                
                # Rotazione (Yaw)
                elif key.char == 'a': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, self.STEP_YAW)) # Sinistra
                elif key.char == 'd': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, -self.STEP_YAW)) # Destra
                
                # Funzioni Rapide
                elif key.char == 't': self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 1.0)) # Decollo auto
                elif key.char == 'g': self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05)) # Atterraggio auto
                
                # Chiusura
                elif key.char == 'q': self.running = False; return False
            
            # COMANDI DIREZIONALI (Frecce - Piano XY)
            if key == keyboard.Key.up: self.sim.setObjectPosition(self.drone, self.drone, (self.STEP_MOVE, 0, 0))    # Avanti
            elif key == keyboard.Key.down: self.sim.setObjectPosition(self.drone, self.drone, (-self.STEP_MOVE, 0, 0)) # Indietro
            elif key == keyboard.Key.left: self.sim.setObjectPosition(self.drone, self.drone, (0, self.STEP_MOVE, 0))  # Sinistra
            elif key == keyboard.Key.right: self.sim.setObjectPosition(self.drone, self.drone, (0, -self.STEP_MOVE, 0)) # Destra
        except:
            pass # Ignora errori di pressione tasti non mappati

    def run(self):
        """Ciclo principale di esecuzione"""
        # Avvio del listener tastiera in un thread separato
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()
        
        while self.running:
            # Otteniamo la telemetria calcolata
            line = self.get_telemetry()
            if line:
                # Stampiamo sulla stessa riga (\r) per un effetto dashboard pulito
                print(f"\r{line}", end="", flush=True)
            
            # ATTESA DI SICUREZZA (0.1s = 10Hz)
            # Fondamentale per non saturare la connessione ZMQ e prevenire i crash
            time.sleep(0.1) 
            
        listener.stop()
        print("\nDisconnessione DJI Mini 3 completata.")

if __name__ == "__main__":
    controller = Drone_Controller()
    controller.run()
