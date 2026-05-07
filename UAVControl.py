import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pynput import keyboard

class Drone_Controller:
    def __init__(self):
        
        # Connessione al simulatore
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.drone = self.sim.getObject('/Quadcopter_target')
        
        # Reset al centro (0, 0, 0.05) per evitare compenetrazioni col suolo
        self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05))
        self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, 0))
        
        # Costanti Geodetiche (WGS84)
        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self.LAT_METERS_PER_DEG = 111319.9
        
        # Costanti di volo
        self.STEP_MOVE = 0.05
        self.STEP_YAW = 0.1
        self.ALT_DECOLLO = 1.0
        
        print("--- Sistema di Navigazione Pronto ---")
        print("Comandi: Frecce (XY), W/S (Z), A/D (Yaw), T/G (Decollo/Atterraggio), Q (Esci)")

    def comandi(self, key):
        try:
            # Movimenti Traslazionali (XY)
            if key == keyboard.Key.up: self.sim.setObjectPosition(self.drone, self.drone, (self.STEP_MOVE, 0, 0))
            elif key == keyboard.Key.down: self.sim.setObjectPosition(self.drone, self.drone, (-self.STEP_MOVE, 0, 0))
            elif key == keyboard.Key.left: self.sim.setObjectPosition(self.drone, self.drone, (0, -self.STEP_MOVE, 0))
            elif key == keyboard.Key.right: self.sim.setObjectPosition(self.drone, self.drone, (0, self.STEP_MOVE, 0))
            
            elif hasattr(key, 'char'):
                # Z-Axis e Rotazioni
                if key.char == 'w': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, self.STEP_MOVE))
                elif key.char == 's': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, -self.STEP_MOVE))
                elif key.char == 'a': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, self.STEP_YAW))
                elif key.char == 'd': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, -self.STEP_YAW))
                elif key.char == 't': self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, self.ALT_DECOLLO))
                elif key.char == 'g': self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05))
                elif key.char == 'q': return False
            
            # Calcolo posizione geodetica corretta
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            lat = self.ref_lat + (pos[0] / self.LAT_METERS_PER_DEG)
            
            # Correzione longitudine in base alla latitudine
            lon_factor = self.LAT_METERS_PER_DEG * math.cos(math.radians(lat))
            lon = self.ref_lon + (pos[1] / lon_factor)
            alt = pos[2] 
            
            print(f"Lat: {lat:.7f} | Lon: {lon:.7f} | Alt: {alt:.2f} m       ", end='\r')
            
        except: 
            pass

    def run(self):
        with keyboard.Listener(on_press=self.comandi) as listener:
            listener.join()

if __name__ == "__main__":
    # Avvio del controller
    controller = Drone_Controller()
    controller.run()
