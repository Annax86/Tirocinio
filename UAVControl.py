from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pynput import keyboard

# Connessione al simulatore
client = RemoteAPIClient()
sim = client.getObject('sim')

# Ottieni l'handle dell'oggetto
target = sim.getObject('/Quadcopter_target')
step = 0.05 

# Apri la console di debug in CoppeliaSim
console_handle = sim.auxiliaryConsoleOpen("Coordinate Drone", 10, 2)

print("Controllo attivo: Frecce (XY), W/S (Z), Q (Esci)")

def on_press(key):
    try:
        # Movimenti
        if key == keyboard.Key.up:
            sim.setObjectPosition(target, target, (step, 0, 0))
        elif key == keyboard.Key.down:
            sim.setObjectPosition(target, target, (-step, 0, 0))
        elif key == keyboard.Key.left:
            sim.setObjectPosition(target, target, (0, -step, 0))
        elif key == keyboard.Key.right:
            sim.setObjectPosition(target, target, (0, step, 0))
        elif hasattr(key, 'char'):
            if key.char == 'w':
                sim.setObjectPosition(target, target, (0, 0, step))
            elif key.char == 's':
                sim.setObjectPosition(target, target, (0, 0, -step))

        # --- PARTE AGGIUNTA PER LE COORDINATE ---
        # Leggiamo la posizione rispetto al 'mondo' (coordinate assolute della scena)
        pos = sim.getObjectPosition(target, sim.handle_world)
        
        # Formattiamo la stringa
        coord_text = f"X: {pos[0]:.3f} | Y: {pos[1]:.3f} | Z: {pos[2]:.3f}"
        
        # Stampiamo sul terminale Python
        print(f"Nuova posizione: {coord_text}")
        
        # Stampiamo sulla console di CoppeliaSim
        sim.auxiliaryConsolePrint(console_handle, f"{coord_text}\n")
        # ----------------------------------------

    except Exception as e:
        print(f"Errore: {e}")

# Listener tastiera
with keyboard.Listener(on_press=on_press) as listener:
    listener.join()