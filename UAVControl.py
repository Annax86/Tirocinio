from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pynput import keyboard

class Drone_Controller:
    def __init__(self):
        self.sim = RemoteAPIClient().getObject('sim')
        self.drone = self.sim.getObject('/Quadcopter_target')
        
        # Costanti di volo
        self.STEP_MOVE = 0.05
        self.STEP_YAW = 0.1
        self.ALT_DECOLLO = 1.0
        
        print("--- Sistema Pronto ---")
        print("Comandi: Frecce (XY), W/S (Z), A/D (Yaw), T/G (Decollo/Atterraggio), Q (Esci)")

    def comandi(self, key):
        try:
            # Movimenti e Rotazioni
            if key == keyboard.Key.up: self.sim.setObjectPosition(self.drone, self.drone, (self.STEP_MOVE, 0, 0))
            elif key == keyboard.Key.down: self.sim.setObjectPosition(self.drone, self.drone, (-self.STEP_MOVE, 0, 0))
            elif key == keyboard.Key.left: self.sim.setObjectPosition(self.drone, self.drone, (0, -self.STEP_MOVE, 0))
            elif key == keyboard.Key.right: self.sim.setObjectPosition(self.drone, self.drone, (0, self.STEP_MOVE, 0))
            elif hasattr(key, 'char'):
                if key.char == 'w': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, self.STEP_MOVE))
                elif key.char == 's': self.sim.setObjectPosition(self.drone, self.drone, (0, 0, -self.STEP_MOVE))
                elif key.char == 'a': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, self.STEP_YAW))
                elif key.char == 'd': self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, -self.STEP_YAW))
                elif key.char == 't': self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, self.ALT_DECOLLO))
                elif key.char == 'g': self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0, 0, 0.05))
                elif key.char == 'q': return False
            
            # Feedback posizione su terminale
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            print(f"X:{pos[0]:.2f} | Y:{pos[1]:.2f} | Z:{pos[2]:.2f}     ", end='\r')
        except: 
            pass

    def run(self):
        with keyboard.Listener(on_press=self.comandi) as listener:
            listener.join()

if __name__ == "__main__":
    Drone_Controller().run()
