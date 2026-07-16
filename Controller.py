"""
DJI Mini 3 — Ground Control Station per CoppeliaSim
=====================================================
Controller da tastiera per il quadricottero simulato, con localizzazione
opzionale basata su marker ArUco (multilaterazione + stima yaw) e navigazione
autonoma verso waypoint (GNSS o ArUco).

Requisiti:
    pip install opencv-python opencv-contrib-python numpy pynput coppeliasim-zmqremote-api

Nota: questo script usa `msvcrt`, quindi funziona solo su Windows.
"""

import math
import re
import time
import threading
import numpy as np
import cv2  # Richiede: pip install opencv-python opencv-contrib-python
import msvcrt  # Libreria nativa Windows per la gestione del buffer di input
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pynput import keyboard

# Riconosce le sequenze di escape ANSI (colori/stili) per poterle ignorare
# quando si calcola la larghezza VISIBILE di una stringa da incorniciare.
_ANSI_ESCAPE_RE = re.compile(r"\033\[[0-9;]*m")


class Drone_Controller:
    # ----------------------------------------------------------------- #
    # --- COSTANTI DI LAYOUT TERMINALE (posizionamento ANSI assoluto) --- #
    # ----------------------------------------------------------------- #
    # L'intera dashboard è UN SOLO riquadro continuo: intestazione,
    # telemetria, debug ArUco, messaggi di sistema e legenda comandi
    # condividono lo stesso bordo, cosi' nessuna riga puo' "galleggiare"
    # fuori dal rettangolo o sconfinare oltre il bordo destro (ogni riga
    # passa da _box_row(), che calcola il padding sulla lunghezza VISIBILE
    # del testo, ignorando i codici colore ANSI).
    ROW_HEADER_TOP = 1
    ROW_HEADER_TITLE = 2
    ROW_HEADER_SEP = 3
    ROW_TELEMETRY = 4
    ROW_ARUCO_DEBUG = 5
    ROW_SEP2 = 6
    ROW_MESSAGE = 7
    ROW_SEP3 = 8
    ROW_LEGEND_START = 9   # le righe della legenda sono calcolate dinamicamente

    # Legenda comandi: una voce per riga (nessuna colonna affiancata), cosi'
    # da avere sempre spazio a sufficienza ed evitare sovrapposizioni di
    # testo indipendentemente da quanto e' lunga la descrizione.
    LEGEND = [
        ("W / S", "Sali / Scendi"),
        ("Frecce ← ↑ → ↓", "Traslazione orizzontale"),
        ("A / D", "Yaw sinistra / destra"),
        ("I / K", "Tilt camera su / giù"),
        ("U / J", "Zoom camera (FOV -/+)"),
        ("T / G", "Decollo / Atterraggio"),
        ("O", "Orbita temporizzata"),
        ("P", "Waypoint (coordinate manuali o localizzazione ArUco)"),
        ("M", "Configura passo di movimento delle frecce"),
        ("V", "Insegui automaticamente un marker ArUco"),
        ("L", "Attiva/disattiva la localizzazione tramite ArUco"),
        ("Z", "Attiva/disattiva il debug ArUco (riga diagnostica)"),
        ("Q", "Esci dal programma"),
    ]

    # Larghezza del contenuto interno del box (senza bordi), calcolata in
    # automatico per contenere comodamente sia la riga di telemetria (la
    # più larga) sia la voce di legenda più lunga, con un margine di respiro.
    BOX_WIDTH = 100

    # Righe calcolate a valle della legenda (dipendono dal numero di voci,
    # quindi non sono più costanti fisse "a occhio": se in futuro si aggiunge
    # una voce alla legenda, tutto il resto del layout si adatta da solo).
    ROW_LEGEND_END = ROW_LEGEND_START + len(LEGEND) - 1
    ROW_BOTTOM_BORDER = ROW_LEGEND_END + 1
    ROW_INPUT_AREA = ROW_BOTTOM_BORDER + 2  # da qui in poi scrivono i thread di input GCS

    def __init__(self):
        """
        Inizializzazione del controllore globale per CoppeliaSim.
        Configura la connessione ZMQ, i sensori e le strutture dati asincrone.
        """
        self._init_palette()

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

                # Risoluzione 512x512 per permettere il rilevamento affidabile di
                # più marker ArUco contemporaneamente e a distanza. A 256x256 ogni
                # marker occupa troppo pochi pixel per essere decodificato
                # correttamente da OpenCV quando ne sono visibili due o più.
                self.sim.setObjectInt32Param(self.vision_sensor, 1002, 512)  # larghezza
                self.sim.setObjectInt32Param(self.vision_sensor, 1003, 512)  # altezza
                self._boot_log("Vision Sensor inizializzato a 60° — risoluzione 512x512.", ok=True)
            except Exception as e:
                self.vision_sensor = None
                self._boot_log(f"Impossibile configurare /visionSensor: {e}", ok=False)

            # 3. PIPELINE ARUCO (Standard Ufficiali OpenCV 4.x)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()

            # Parametri ottimizzati per rilevamento di più marker simultanei,
            # a distanza e con illuminazione simulata.

            # Soglia adattiva: finestra più grande per cogliere marker piccoli/lontani
            self.aruco_params.adaptiveThreshWinSizeMin = 3
            self.aruco_params.adaptiveThreshWinSizeMax = 53
            self.aruco_params.adaptiveThreshWinSizeStep = 4
            self.aruco_params.adaptiveThreshConstant = 7

            # Accetta marker anche se occupano pochi pixel (utile a distanza)
            self.aruco_params.minMarkerPerimeterRate = 0.01
            self.aruco_params.maxMarkerPerimeterRate = 4.0

            # Tolleranza geometrica maggiore: marker visti di lato/prospettiva
            self.aruco_params.polygonalApproxAccuracyRate = 0.05
            self.aruco_params.minCornerDistanceRate = 0.01
            self.aruco_params.minMarkerDistanceRate = 0.01

            # Bit detection: meno restrittivo per immagini sintetiche con anti-aliasing
            self.aruco_params.minOtsuStdDev = 3.0
            self.aruco_params.perspectiveRemovePixelPerCell = 8
            self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.13
            self.aruco_params.maxErroneousBitsInBorderRate = 0.35
            self.aruco_params.errorCorrectionRate = 0.6

            # Corner refinement per maggiore precisione sub-pixel (utile per yaw)
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.cornerRefinementWinSize = 5
            self.aruco_params.cornerRefinementMaxIterations = 30
            self.aruco_params.cornerRefinementMinAccuracy = 0.01
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

            # 4. RESET DELLO STATO INIZIALE DEL VELIVOLO NELLO SPAZIO 3D
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (0.00, -3.700, 0.05))
            self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, 0.0))

            # Reset del segnale del servomotore Gimbal della telecamera (0° = orizzontale)
            self.camera_tilt = 0.0
            self.sim.setFloatSignal("cameraTilt", self.camera_tilt)

        except Exception as e:
            print(f"\033[1;31mErrore critico durante l'inizializzazione: {e}\033[0m")
            raise SystemExit(1)

        # --- PARAMETRI DI NAVIGAZIONE CINEMATICA E GEODETICA ---
        self.ref_lat, self.ref_lon = 0.0, 0.0
        self.LAT_METERS_PER_DEG = 111319.9

        self.STEP_MOVE = 0.08
        self.STEP_YAW = math.radians(5)
        self.TILT_STEP = math.radians(2)
        self.ARRIVED_THRESH = 0.10  # Soglia a 10 cm per un aggancio ultra-preciso

        # Memoria condivisa thread-safe per il salvataggio dei waypoint dell'autopilota
        self._lock = threading.Lock()
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = 0.8

        # Registri per la localizzazione stimata tramite computer vision inversa
        self.aruco_pos_x = 0.0
        self.aruco_pos_y = 0.0
        self.has_aruco_pos = False
        self.aruco_localization_marker_count = 0  # numero di marker usati nell'ultima stima
        self.aruco_yaw = 0.0
        self.has_aruco_yaw = False

        # Mappa runtime dei marker ArUco: id -> (x, y) posizione assoluta stimata.
        # Costruita "al volo": la prima volta che un marker viene visto, la sua
        # posizione viene calcolata dalla posizione nota del drone in quell'istante
        # + direzione di vista + distanza stimata otticamente, e viene fissata.
        # Da quel momento il marker funge da beacon noto per le localizzazioni successive.
        self.aruco_map = {}

        # Configurazione dinamica dello step di movimento frecce (impostata con 'm')
        self.movement_step_configured = False
        self.movement_pulse_ms = None
        self.movement_power = None

        # Waypoint navigation basata sulla mappa ArUco runtime (alternativa
        # all'inserimento manuale di coordinate, attivabile con lo stesso tasto 'p')
        self.waypoint_aruco_mode = False
        self.aruco_debug_mode = False  # 'z': stampa info diagnostiche sul rilevamento ArUco

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

        # Disegna l'interfaccia iniziale pulita (schermo intero + dashboard fissa)
        self._clear_screen()
        self.repaint_header()

    # ----------------------------------------------------------------- #
    # --- ESTETICA / INTERFACCIA TERMINALE --------------------------- #
    # ----------------------------------------------------------------- #
    def _init_palette(self):
        """Tabella colori/ badge ANSI centralizzata, riusata da tutta l'interfaccia."""
        self.C_RESET = "\033[0m"
        self.C_DIM = "\033[2m"
        self.C_BOLD = "\033[1m"

        self.C_BORDER = "\033[1;34m"     # blu per i bordi del box
        self.C_TITLE = "\033[1;36m"      # ciano per titoli
        self.C_TEXT = "\033[0;37m"       # bianco/grigio per testo normale
        self.C_KEY = "\033[1;33m"        # giallo per i tasti nella legenda
        self.C_OK = "\033[1;32m"
        self.C_ERR = "\033[1;31m"
        self.C_WARN = "\033[1;33m"

        # Badge di stato compatti (anti line-wrap)
        self.C_GNSS = "\033[1;37;42m GNSS \033[0m"
        self.C_ARUCO = "\033[1;37;46m ARUCO \033[0m"
        self.C_MANUAL = "\033[1;37;44m MANU \033[0m"
        self.C_AUTO = "\033[1;37;43m AUTO \033[0m"
        self.C_ALERT = "\033[1;37;41m ALER \033[0m"

    @staticmethod
    def _clear_screen():
        print("\033[2J\033[H\033[?25l", end="", flush=True)  # pulisce + nasconde il cursore

    @staticmethod
    def _show_cursor():
        print("\033[?25h", end="", flush=True)

    def _boot_log(self, msg, ok=True):
        """Log di avvio, stampato prima che la dashboard fissa sia disegnata."""
        colore = self.C_OK if ok else self.C_ERR
        tag = "INFO" if ok else "ERROR"
        print(f"{colore}[{tag}] {msg}{self.C_RESET}")

    @staticmethod
    def _visible_len(text):
        """Lunghezza di `text` come appare a schermo, ignorando i codici colore ANSI."""
        return len(_ANSI_ESCAPE_RE.sub("", text))

    def _box_row(self, text, color=""):
        """
        Costruisce una riga del box con i bordi sempre allineati. Il padding
        viene calcolato sulla lunghezza VISIBILE del testo (i codici colore
        ANSI vengono ignorati), quindi qualunque riga passi da qui — che
        contenga colori o meno — resta sempre correttamente racchiusa tra i
        due bordi verticali, senza mai sconfinare né lasciare buchi.
        Se il testo è più lungo del box, viene troncato in modo sicuro
        (ignorando i codici ANSI) invece di rompere l'allineamento.
        """
        lunghezza = self._visible_len(text)
        if lunghezza > self.BOX_WIDTH:
            testo_semplice = _ANSI_ESCAPE_RE.sub("", text)
            text = testo_semplice[: self.BOX_WIDTH - 1] + "…"
            lunghezza = self.BOX_WIDTH
        pad = " " * (self.BOX_WIDTH - lunghezza)
        return (f"{self.C_BORDER}║{self.C_RESET}{color} {text}{pad} {self.C_RESET}"
                f"{self.C_BORDER}║{self.C_RESET}")

    def _box_border(self, kind="top"):
        """Riga di bordo orizzontale: 'top', 'mid' (separatore) o 'bottom'."""
        larghezza = self.BOX_WIDTH + 2
        if kind == "top":
            return f"{self.C_BORDER}╔{'═' * larghezza}╗{self.C_RESET}"
        if kind == "bottom":
            return f"{self.C_BORDER}╚{'═' * larghezza}╝{self.C_RESET}"
        return f"{self.C_BORDER}╠{'═' * larghezza}╣{self.C_RESET}"

    def repaint_header(self):
        """
        Ridisegna l'intera dashboard come UN SOLO riquadro continuo (intestazione,
        telemetria, debug ArUco, messaggi di sistema e legenda comandi), usando
        posizionamento ANSI assoluto, senza mai pulire lo schermo durante il volo
        (evita flickering). Le righe dinamiche (telemetria, debug, messaggi)
        vengono qui solo "riservate" vuote: il loro contenuto viene aggiornato
        altrove (get_telemetry_string/run, process_all_aruco_detections,
        print_msg), sempre passando da _box_row() per restare dentro i bordi.
        """
        print(f"\033[{self.ROW_HEADER_TOP};1H{self._box_border('top')}\033[K")

        titolo = "DJI MINI 3 — GROUND CONTROL STATION"
        print(f"\033[{self.ROW_HEADER_TITLE};1H"
              + self._box_row(titolo, self.C_TITLE + self.C_BOLD)
              + "\033[K")
        print(f"\033[{self.ROW_HEADER_SEP};1H{self._box_border('mid')}\033[K")

        # Righe di telemetria/debug ArUco: riservate vuote, riempite dal loop
        # principale e da process_all_aruco_detections rispettivamente.
        print(f"\033[{self.ROW_TELEMETRY};1H{self._box_row('')}\033[K")
        print(f"\033[{self.ROW_ARUCO_DEBUG};1H{self._box_row('')}\033[K")
        print(f"\033[{self.ROW_SEP2};1H{self._box_border('mid')}\033[K")

        # Riga messaggi di sistema: riservata vuota, riempita da print_msg().
        print(f"\033[{self.ROW_MESSAGE};1H{self._box_row('')}\033[K")
        print(f"\033[{self.ROW_SEP3};1H{self._box_border('mid')}\033[K")

        # Legenda comandi: una voce per riga, tasto in evidenza + descrizione.
        # La larghezza del box (BOX_WIDTH) è ampia apposta, quindi qui c'è
        # sempre spazio abbondante e nessuna voce può più sparire (in
        # precedenza le ultime voci venivano tagliate perché lo spazio
        # allocato alla legenda era fisso e insufficiente).
        row = self.ROW_LEGEND_START
        for tasto, descrizione in self.LEGEND:
            testo = f"{self.C_KEY}{tasto:<16}{self.C_RESET}{descrizione}"
            print(f"\033[{row};1H" + self._box_row(testo) + "\033[K")
            row += 1

        print(f"\033[{self.ROW_BOTTOM_BORDER};1H{self._box_border('bottom')}\033[K")

    def print_msg(self, msg):
        """Stampa un messaggio di sistema sempre alla riga dedicata, incorniciato
        come tutto il resto della dashboard, senza mai sovrascrivere le altre
        sezioni (header, telemetria, debug, legenda)."""
        print(f"\033[{self.ROW_MESSAGE};1H" + self._box_row(msg) + "\033[K", flush=True)

    def _clear_aruco_debug_row(self):
        print(f"\033[{self.ROW_ARUCO_DEBUG};1H" + self._box_row("") + "\033[K", end="", flush=True)

    # --- EQUAZIONE DI COMPENSAZIONE OTTICA PER LO ZOOM (TESI) ---
    def calculate_compensated_distance(self, lunghezza_lato):
        # Costante tarata per risoluzione 512x512: metà altezza (256px) * tan(30°)
        costante_calibrazione = 256.0 * math.tan(math.radians(30))
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

    # --- GESTIONE CENTRALIZZATA DELLE MODALITÀ AUTONOME ---
    def reset_autonomous_modes(self, keep=()):
        """
        Disattiva tutte le modalità di volo autonome/semi-autonome, tranne quelle
        elencate in `keep`. Centralizzare questo reset evita che vecchie modalità
        restino "orfane" attive in background quando se ne attiva una nuova
        (bug presente nella versione precedente, es. il tasto 'v' non azzerava
        waypoint_aruco_mode).
        """
        tutte = ("takeoff_mode", "land_mode", "orbit_mode", "waypoint_mode",
                 "waypoint_aruco_mode", "aruco_mode", "localization_mode")
        for nome in tutte:
            if nome not in keep:
                setattr(self, nome, False)
        if "localization_mode" not in keep:
            self.has_aruco_pos = False
            self.has_aruco_yaw = False
            self.aruco_localization_marker_count = 0
        if "aruco_mode" not in keep and "waypoint_mode" not in keep:
            self.is_searching_aruco = False

    def get_telemetry_string(self):
        """Genera la riga di telemetria estetica super-compatta per evitare il line-wrap."""
        try:
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)

            yaw_deg = math.degrees(ori[2])
            if abs(yaw_deg) < 0.05:
                yaw_deg = 0.0

            if self.localization_mode and self.has_aruco_pos:
                x_val, y_val = self.aruco_pos_x, self.aruco_pos_y
                n = self.aruco_localization_marker_count
                badge_sensore = self.C_ARUCO if n >= 2 else "\033[1;30;46m AR_1M \033[0m"

                if self.has_aruco_yaw:
                    yaw_deg = math.degrees(self.aruco_yaw)
                    if abs(yaw_deg) < 0.05:
                        yaw_deg = 0.0
            else:
                x_val, y_val = pos[0], pos[1]
                badge_sensore = self.C_GNSS

            if self.takeoff_mode:
                badge_modo = self.C_ALERT + " TAKEOFF"
            elif self.land_mode:
                badge_modo = self.C_ALERT + " LANDING"
            elif self.orbit_mode:
                badge_modo = self.C_AUTO + " ORBIT"
            elif self.waypoint_aruco_mode:
                badge_modo = self.C_AUTO + " WP_ARUCO"
            elif self.waypoint_mode:
                badge_modo = self.C_AUTO + " WP_NAV"
            elif self.aruco_mode:
                badge_modo = self.C_AUTO + " AR_SCAN"
            elif self.localization_mode:
                # Numero REALE di marker unici usati nell'ultima stima (bug corretto:
                # in precedenza poteva essere raddoppiato da rilevamenti duplicati
                # dello stesso ID). Vedi process_all_aruco_detections().
                badge_modo = self.C_ARUCO + f" LOC_ON({self.aruco_localization_marker_count})"
            else:
                badge_modo = self.C_MANUAL + " HOVER"

            if self.movement_step_configured:
                step_info = f"STEP:{self.STEP_MOVE:.3f}m"
            else:
                step_info = "\033[1;33mSTEP:?\033[0m"

            stato_volo = f"{self.C_OK}AIRBORNE{self.C_RESET}" if self.is_airborne else f"{self.C_DIM}GROUNDED{self.C_RESET}"

            return (
                f"{badge_sensore} │ X:{x_val:5.2f} │ Y:{y_val:5.2f} │ Z:{pos[2]:4.2f} │ "
                f"Ψ:{yaw_deg:5.1f}° │ {badge_modo} │ {step_info} │ {stato_volo}"
            )
        except Exception:
            return None

    # --- OPERAZIONI ASINCRONE DELLA GROUND CONTROL STATION (THREAD) ---
    def _gcs_prompt_begin(self, titolo):
        """Prepara l'area di input GCS: posiziona il cursore sotto la dashboard
        fissa, mostra il cursore e stampa l'intestazione del comando."""
        self.key_pressed = None
        self.gcs_input_active = True
        print(f"\033[{self.ROW_INPUT_AREA};1H\033[J\033[?25h", end="")  # pulisce da qui in giù, mostra cursore
        print(self.C_TITLE + "═" * 55)
        print(f" [GCS COMMAND] {titolo}")
        print("═" * 55 + self.C_RESET)

    def _gcs_prompt_end(self):
        """Chiude il prompt GCS: nasconde di nuovo il cursore e ridisegna la dashboard."""
        time.sleep(1.5)
        print("\033[?25l", end="")
        self._clear_screen()
        self.repaint_header()
        self.key_pressed = None
        self.gcs_input_active = False

    def ask_coordinates_thread(self):
        self._gcs_prompt_begin("WAYPOINT NAVIGATION")
        try:
            scelta = input(" -> Sorgente posizione [1] GNSS/coordinate vere  [2] Localizzazione ArUco: ").strip()

            x = float(input(" -> Coordinata Target X (metri): "))
            y = float(input(" -> Coordinata Target Y (metri): "))
            z = float(input(" -> Quota di Volo Target Z (metri): "))
            self.set_target_coordinates(x, y, z)

            if scelta == "2":
                self.reset_autonomous_modes()
                self.waypoint_aruco_mode = True
                print(f"{self.C_OK} [✓] Rotta calcolata. Navigazione basata su localizzazione ArUco agganciata.{self.C_RESET}")
            else:
                self.reset_autonomous_modes()
                self.waypoint_mode = True
                print(f"{self.C_OK} [✓] Rotta calcolata. Autopilota agganciato.{self.C_RESET}")
        except ValueError:
            print(f"{self.C_ERR} [X] Input non numerico. Procedura abortita.{self.C_RESET}")
            self.waypoint_mode = False
            self.waypoint_aruco_mode = False

        self._gcs_prompt_end()

    def ask_movement_command_thread(self):
        """
        Chiede all'utente i parametri (durata impulso e potenza) usati per calcolare
        lo STEP_MOVE applicato a ogni pressione delle frecce direzionali.
        Non muove il drone: si limita a configurare il passo di spostamento.
        """
        self._gcs_prompt_begin("CONFIGURAZIONE PASSO DI MOVIMENTO FRECCE")
        try:
            tempo = float(input(" -> Durata impulso elettrico (millisecondi): "))
            potenza = float(input(" -> Coefficiente Potenza applicata (es. 0.05): "))

            tempo_s = tempo / 1000.0
            spazio = tempo_s * potenza * 10.0

            self.movement_pulse_ms = tempo
            self.movement_power = potenza
            self.STEP_MOVE = spazio
            self.movement_step_configured = True

            print(f"{self.C_OK} [✓] Passo di movimento impostato: {spazio:.4f} metri per pressione freccia.{self.C_RESET}")
        except ValueError:
            print(f"{self.C_ERR} [X] Dati immessi errati. Configurazione non modificata.{self.C_RESET}")

        self._gcs_prompt_end()

    def ask_orbit_command_thread(self):
        self._gcs_prompt_begin("CONFIGURAZIONE TRAIETTORIA CIRCOLARE")
        try:
            durata = float(input(" -> Durata temporizzatore orbita (secondi): "))
            pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)

            self.orbit_center = [pos[0], pos[1], pos[2]]
            self.orbit_angle = 0.0
            self.orbit_duration = durata
            self.orbit_start_time = time.time()
            self.orbit_mode = True
            print(f"{self.C_OK} [✓] Orbita agganciata per {durata}s.{self.C_RESET}")
        except ValueError:
            print(f"{self.C_ERR} [X] Valore temporale errato.{self.C_RESET}")
            self.orbit_mode = False

        self._gcs_prompt_end()

    # --- FUNZIONI DI DINAMICA INTERNA ED FISICA DI VOLO ---
    def gradual_takeoff(self):
        if not self.takeoff_mode:
            return
        try:
            drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            current_alt = drone_pos[2]
            dz = self.takeoff_target_alt - current_alt

            if abs(dz) < 0.05:
                self.sim.setObjectPosition(self.drone, self.sim.handle_world,
                                            (drone_pos[0], drone_pos[1], self.takeoff_target_alt))
                self.takeoff_mode = False
                self.is_airborne = True
                self.print_msg(f"{self.C_OK}[SYSTEM] Decollo ultimato. Velivolo stabilizzato a {self.takeoff_target_alt}m.{self.C_RESET}")
                return

            step = self.takeoff_speed * 0.05 if dz > 0 else -self.takeoff_speed * 0.05
            if abs(step) > abs(dz):
                step = dz
            self.sim.setObjectPosition(self.drone, self.sim.handle_world,
                                        (drone_pos[0], drone_pos[1], current_alt + step))
        except Exception:
            self.takeoff_mode = False

    def gradual_landing(self):
        if not self.land_mode:
            return
        try:
            drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
            current_alt = drone_pos[2]
            if current_alt <= 0.052:
                self.land_mode = self.is_airborne = False
                self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], 0.05))
                self.print_msg(f"{self.C_ERR}[SYSTEM] Touchdown rilevato. Motori spenti.{self.C_RESET}")
                return
            velocita_effettiva = self.land_speed
            if current_alt <= 0.25:
                velocita_effettiva = max(self.land_speed * (current_alt / 0.25), 0.04)
            step = velocita_effettiva * 0.05
            nuova_alt = max(current_alt - step, 0.05)
            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], nuova_alt))
        except Exception:
            self.land_mode = False

    def update_orbit(self):
        if not self.orbit_mode:
            return
        if self.orbit_duration is not None:
            if time.time() - self.orbit_start_time >= self.orbit_duration:
                self.orbit_mode = False
                self.print_msg(f"{self.C_WARN}[SYSTEM] Timer orbita scaduto.{self.C_RESET}")
                return

        self.orbit_angle -= self.orbit_speed
        new_x = self.orbit_center[0] + self.orbit_radius * math.cos(self.orbit_angle)
        new_y = self.orbit_center[1] + self.orbit_radius * math.sin(self.orbit_angle)
        target_yaw = self.orbit_angle + math.pi
        self.sim.setObjectPosition(self.drone, self.sim.handle_world, (new_x, new_y, self.orbit_center[2]))
        self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (0, 0, target_yaw))

    # --- SOTTO-SISTEMI DI VISIONE COMPUTAZIONALE OPENCV ---
    def process_aruco_detection(self, target_id):
        if not self.vision_sensor:
            return None, None
        try:
            img_buffer, resolution = self.sim.getVisionSensorImg(self.vision_sensor)
            if not img_buffer or len(img_buffer) == 0:
                return None, None

            img = np.frombuffer(img_buffer, dtype=np.uint8)
            img.shape = (resolution[1], resolution[0], 3)
            img = cv2.flip(img, 0)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            if ids is not None:
                for idx, marker_id in enumerate(ids.flatten()):
                    if marker_id == target_id:
                        return corners[idx][0], resolution
        except Exception:
            pass
        return None, None

    def process_all_aruco_detections(self):
        """
        Rileva TUTTI i marker ArUco visibili in un singolo frame.
        Necessario per la trilaterazione: servono almeno due marker
        contemporaneamente nel campo visivo della telecamera.

        FIX: con parametri di rilevamento molto permissivi (necessari per
        cogliere marker piccoli/lontani), OpenCV può talvolta restituire DUE
        candidati distinti per lo stesso marker fisico (stesso ID, contorni
        leggermente diversi). Senza deduplica, un marker visto due volte
        veniva contato due volte a valle (in update_aruco_map, nella
        multilaterazione e nel badge "LOC_ON(n)"), facendo apparire un
        conteggio doppio rispetto ai marker realmente visibili (es. 3 marker
        reali -> "LOC_ON(6)", 4 marker reali -> "LOC_ON(8)"). Qui si tiene,
        per ogni ID, solo il rilevamento con il lato del contorno più grande
        (il più affidabile), garantendo un solo risultato per marker.

        Ritorna (lista_di_tuple(marker_id, corners_4x2), resolution).
        """
        if not self.vision_sensor:
            return [], None
        try:
            img_buffer, resolution = self.sim.getVisionSensorImg(self.vision_sensor)
            if not img_buffer or len(img_buffer) == 0:
                return [], None

            img = np.frombuffer(img_buffer, dtype=np.uint8)
            img.shape = (resolution[1], resolution[0], 3)
            img = cv2.flip(img, 0)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

            if self.aruco_debug_mode:
                n_rilevati = len(ids.flatten()) if ids is not None else 0
                n_scartati = len(rejected) if rejected is not None else 0
                ids_trovati = list(int(i) for i in ids.flatten()) if ids is not None else []
                riga_debug = (f"{self.C_TITLE}[ARUCO DEBUG]{self.C_RESET} "
                               f"Rilevati:{n_rilevati} ID:{ids_trovati} | Scartati:{n_scartati} | "
                               f"Res:{resolution}")
                print(f"\033[{self.ROW_ARUCO_DEBUG};1H" + self._box_row(riga_debug) + "\033[K",
                      end="", flush=True)

            # --- DEDUPLICA PER ID: tiene solo il rilevamento più affidabile per marker ---
            migliori = {}  # marker_id -> (corners, lunghezza_lato)
            if ids is not None:
                for idx, raw_id in enumerate(ids.flatten()):
                    marker_id = int(raw_id)
                    c = corners[idx][0]
                    lunghezza_lato = math.hypot(c[0][0] - c[1][0], c[0][1] - c[1][1])
                    if marker_id not in migliori or lunghezza_lato > migliori[marker_id][1]:
                        migliori[marker_id] = (c, lunghezza_lato)

            risultati = [(marker_id, dati[0]) for marker_id, dati in migliori.items()]
            return risultati, resolution
        except Exception as e:
            if self.aruco_debug_mode:
                riga_debug = f"{self.C_ERR}[ARUCO DEBUG] Eccezione: {e}{self.C_RESET}"
                print(f"\033[{self.ROW_ARUCO_DEBUG};1H" + self._box_row(riga_debug) + "\033[K",
                      end="", flush=True)
            return [], None

    def estimate_marker_world_position(self, lunghezza_lato, drone_pos, angolo_vista_reale):
        """
        Stima la posizione assoluta di un marker NON ancora mappato, usando
        esclusivamente dati noti al drone in quell'istante: la propria posizione
        corrente (nota) e la distanza/direzione stimate dalla telecamera.
        Questo è il passo di "prima osservazione" che fissa il marker in mappa.
        """
        distanza_stimata = self.calculate_compensated_distance(lunghezza_lato)
        mx = drone_pos[0] + (distanza_stimata * math.cos(angolo_vista_reale))
        my = drone_pos[1] + (distanza_stimata * math.sin(angolo_vista_reale))
        return mx, my, distanza_stimata

    def update_aruco_map(self, rilevamenti, drone_pos, angolo_vista_reale, resolution=None):
        """
        Aggiorna la mappa runtime dei marker ArUco. Se un marker viene visto per
        la prima volta, la sua posizione viene stimata dalla telecamera e fissata
        in mappa (diventa un beacon noto per le localizzazioni successive).

        Ogni marker viene mappato usando il suo angolo apparente INDIVIDUALE
        nell'immagine (quanto è spostato rispetto al centro del frame), non la
        direzione generica della prua del drone che è identica per tutti i marker
        visti contemporaneamente — quella causerebbe posizioni coincidenti e
        fallimento della multilaterazione.

        Grazie alla deduplica applicata a monte in process_all_aruco_detections,
        `rilevamenti` contiene al massimo UNA voce per ogni ID fisicamente
        visibile, quindi `beacon_osservati` non può più essere gonfiato da
        duplicati dello stesso marker.
        """
        beacon_osservati = []
        larghezza_px = resolution[0] if resolution is not None else 512

        for marker_id, corners in rilevamenti:
            lunghezza_lato = math.hypot(corners[0][0] - corners[1][0], corners[0][1] - corners[1][1])
            if lunghezza_lato < 15.0:
                continue

            distanza_stimata = self.calculate_compensated_distance(lunghezza_lato)
            centro_px_x = sum(c[0] for c in corners) / 4.0

            if marker_id not in self.aruco_map:
                # Angolo apparente individuale del marker rispetto all'asse ottico:
                # un marker a sinistra del centro ha offset negativo, a destra positivo.
                offset_normalizzato = (centro_px_x - larghezza_px / 2.0) / (larghezza_px / 2.0)
                angolo_offset = offset_normalizzato * (self.camera_fov / 2.0)
                angolo_marker = angolo_vista_reale + angolo_offset

                mx = drone_pos[0] + (distanza_stimata * math.cos(angolo_marker))
                my = drone_pos[1] + (distanza_stimata * math.sin(angolo_marker))
                self.aruco_map[marker_id] = (mx, my)

            mx, my = self.aruco_map[marker_id]
            beacon_osservati.append((marker_id, mx, my, distanza_stimata, centro_px_x))
        return beacon_osservati

    def estimate_yaw_from_markers(self, beacon_osservati, resolution):
        """
        Stima il yaw assoluto del drone confrontando, per ogni coppia di beacon
        mappati e visibili, l'angolo apparente nell'immagine (dedotto dalla loro
        posizione orizzontale in pixel tramite il FOV della telecamera) con
        l'angolo assoluto reale tra le loro posizioni note in mappa.

        Principio: se conosco dove DOVREBBERO apparire due punti noti (mappa) e
        dove APPAIONO davvero nell'immagine, la differenza tra i due angoli mi dà
        l'orientamento assoluto della telecamera (e quindi del drone).
        """
        if resolution is None or len(beacon_osservati) < 2:
            return None

        larghezza_px = resolution[0]
        meta_fov = self.camera_fov / 2.0

        stime_yaw = []
        pesi = []
        n = len(beacon_osservati)
        for i in range(n):
            for j in range(i + 1, n):
                id_i, xi, yi, ri, pxi = beacon_osservati[i]
                id_j, xj, yj, rj, pxj = beacon_osservati[j]

                offset_i = (pxi - larghezza_px / 2.0) / (larghezza_px / 2.0)
                offset_j = (pxj - larghezza_px / 2.0) / (larghezza_px / 2.0)
                angolo_apparente_i = offset_i * meta_fov
                angolo_apparente_j = offset_j * meta_fov

                angolo_reale_assoluto = math.atan2(yj - yi, xj - xi)

                angolo_apparente_relativo = angolo_apparente_j - angolo_apparente_i
                yaw_stimato = angolo_reale_assoluto - angolo_apparente_relativo - (math.pi / 2)

                yaw_stimato = math.atan2(math.sin(yaw_stimato), math.cos(yaw_stimato))

                peso = 1.0 / max((ri + rj) / 2.0, 0.05)
                stime_yaw.append(yaw_stimato)
                pesi.append(peso)

        if not stime_yaw:
            return None

        somma_sin = sum(p * math.sin(y) for y, p in zip(stime_yaw, pesi))
        somma_cos = sum(p * math.cos(y) for y, p in zip(stime_yaw, pesi))
        return math.atan2(somma_sin, somma_cos)

    def multilateration_weighted(self, beacons, fallback_xy):
        """
        Multilaterazione pesata (least-squares) con N >= 2 beacon a posizione nota
        (presi dalla mappa runtime) e relative distanze stimate otticamente.
        Più marker vengono usati, più la stima è robusta al rumore di singola misura.
        I beacon più vicini (distanza stimata minore) pesano di più, perché la
        stima di distanza ottica è più precisa a corto raggio.

        Nota: `beacons` non contiene più duplicati dello stesso ID (deduplicati
        a monte), quindi len(beacons) qui rispecchia il vero numero di marker
        fisicamente distinti osservati in questo frame.
        """
        if len(beacons) < 2:
            return None, 0

        beacons_ord = sorted(beacons, key=lambda b: b[3])
        x0, y0, r0 = beacons_ord[0][1], beacons_ord[0][2], beacons_ord[0][3]

        A_rows = []
        b_rows = []
        weights = []
        for marker_id, xi, yi, ri, _px in beacons_ord[1:]:
            A_rows.append([2 * (xi - x0), 2 * (yi - y0)])
            b_rows.append((r0**2 - ri**2) + (xi**2 - x0**2) + (yi**2 - y0**2))
            distanza_media = (r0 + ri) / 2.0
            weights.append(1.0 / max(distanza_media, 0.05))

        A = np.array(A_rows, dtype=float)
        b = np.array(b_rows, dtype=float)
        W = np.diag(weights)

        try:
            AtW = A.T @ W
            soluzione = np.linalg.solve(AtW @ A, AtW @ b)
            return (float(soluzione[0]), float(soluzione[1])), len(beacons_ord)
        except np.linalg.LinAlgError:
            return None, 0

    # --- CORE ENGINE DI NAVIGAZIONE AUTONOMA ---
    def follow_target(self):
        try:
            if self.localization_mode:
                rilevamenti, resolution = self.process_all_aruco_detections()

                drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                drone_ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
                angolo_vista_reale = drone_ori[2] + (math.pi / 2)

                beacon_osservati = self.update_aruco_map(rilevamenti, drone_pos, angolo_vista_reale, resolution)

                if len(beacon_osservati) >= 2:
                    fallback_xy = (drone_pos[0], drone_pos[1])
                    soluzione, n_usati = self.multilateration_weighted(beacon_osservati, fallback_xy)
                    if soluzione is not None:
                        self.aruco_pos_x, self.aruco_pos_y = soluzione
                        self.has_aruco_pos = True
                        self.aruco_localization_marker_count = n_usati
                    else:
                        self.has_aruco_pos = False
                        self.aruco_localization_marker_count = 0

                    yaw_stimato = self.estimate_yaw_from_markers(beacon_osservati, resolution)
                    if yaw_stimato is not None:
                        self.aruco_yaw = yaw_stimato
                        self.has_aruco_yaw = True
                    else:
                        self.has_aruco_yaw = False

                elif len(beacon_osservati) == 1:
                    marker_id, mx, my, distanza_stimata, _px = beacon_osservati[0]
                    self.aruco_pos_x = mx - (distanza_stimata * math.cos(angolo_vista_reale))
                    self.aruco_pos_y = my - (distanza_stimata * math.sin(angolo_vista_reale))
                    self.has_aruco_pos = True
                    self.aruco_localization_marker_count = 1
                    self.has_aruco_yaw = False
                else:
                    self.has_aruco_pos = False
                    self.aruco_localization_marker_count = 0
                    self.has_aruco_yaw = False
                return

            if self.waypoint_aruco_mode:
                rilevamenti, _res = self.process_all_aruco_detections()
                drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                drone_ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
                angolo_vista_reale = drone_ori[2] + (math.pi / 2)

                beacon_osservati = self.update_aruco_map(rilevamenti, drone_pos, angolo_vista_reale, _res)

                stima_xy = None
                if len(beacon_osservati) >= 2:
                    soluzione, n_usati = self.multilateration_weighted(beacon_osservati, (drone_pos[0], drone_pos[1]))
                    if soluzione is not None:
                        stima_xy = soluzione
                        self.aruco_localization_marker_count = n_usati
                elif len(beacon_osservati) == 1:
                    marker_id, mx, my, distanza_stimata, _px = beacon_osservati[0]
                    stima_xy = (mx - (distanza_stimata * math.cos(angolo_vista_reale)),
                                my - (distanza_stimata * math.sin(angolo_vista_reale)))
                    self.aruco_localization_marker_count = 1

                if stima_xy is None:
                    return

                self.aruco_pos_x, self.aruco_pos_y = stima_xy
                self.has_aruco_pos = True

                tgt_x, tgt_y, tgt_z = self.get_target_coordinates()
                diff_x = tgt_x - stima_xy[0]
                diff_y = tgt_y - stima_xy[1]
                distanza_target = math.sqrt(diff_x**2 + diff_y**2)

                if distanza_target <= self.ARRIVED_THRESH:
                    self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], tgt_z))
                    self.waypoint_aruco_mode = False
                    self.print_msg(f"{self.C_OK}[GCS] Waypoint (localizzazione ArUco) raggiunto.{self.C_RESET}")
                    return

                passo = min(self.STEP_MOVE, distanza_target)
                angolo_target = math.atan2(diff_y, diff_x)
                move_x = passo * math.cos(angolo_target)
                move_y = passo * math.sin(angolo_target)
                self.sim.setObjectPosition(self.drone, self.sim.handle_world,
                                            (drone_pos[0] + move_x, drone_pos[1] + move_y, tgt_z))
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

                if dist < self.ARRIVED_THRESH:
                    self.sim.setObjectPosition(self.drone, self.sim.handle_world, (tgt_x, tgt_y, tgt_z))
                    self.waypoint_mode = False
                    self.key_pressed = None

                    if self.is_searching_aruco:
                        self.print_msg(f"{self.C_OK}[MISSION] Pianta {self.aruco_path[self.current_path_index]} Raggiunta!{self.C_RESET}")
                        if self.current_path_index < len(self.aruco_path) - 1:
                            self.current_path_index += 1
                        self.is_searching_aruco = False
                    else:
                        self.print_msg(f"{self.C_OK}[GCS] Navigazione completata.{self.C_RESET}")
                    return

                x = math.sin(dlam) * math.cos(phi2)
                y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
                target_bearing = (math.degrees(math.atan2(x, y)) + 360) % 360

                cur_yaw_deg = (math.degrees(ori[2]) + 90 + 360) % 360
                err_ang = target_bearing - cur_yaw_deg
                if err_ang > 180:
                    err_ang -= 360
                elif err_ang < -180:
                    err_ang += 360

                ANGLE_TOLERANCE = 5.0
                aligned = False

                if abs(err_ang) <= ANGLE_TOLERANCE:
                    aligned = True
                else:
                    step_yaw = math.degrees(self.STEP_YAW) if err_ang > 0 else -math.degrees(self.STEP_YAW)
                    if abs(step_yaw) > abs(err_ang):
                        step_yaw = err_ang
                    nuovo_yaw = math.radians(math.degrees(ori[2]) + step_yaw)
                    self.sim.setObjectOrientation(self.drone, self.sim.handle_world, (ori[0], ori[1], nuovo_yaw))

                move_x, move_y = 0.0, 0.0
                if aligned:
                    if dist > 0.50:
                        speed_modifier = self.STEP_MOVE
                    else:
                        speed_modifier = self.STEP_MOVE * (dist / 0.50)
                        speed_modifier = max(speed_modifier, 0.005)

                    angolo_verso_target = math.atan2(tgt_y - pos[1], tgt_x - pos[0])
                    move_x = speed_modifier * math.cos(angolo_verso_target)
                    move_y = speed_modifier * math.sin(angolo_verso_target)

                self.sim.setObjectPosition(self.drone, self.sim.handle_world, (pos[0] + move_x, pos[1] + move_y, pos[2]))

            elif self.aruco_mode:
                # Aggancia QUALSIASI marker ArUco visibile nel campo della telecamera,
                # indipendentemente dal suo ID. Tra più marker visibili, si privilegia
                # quello con il lato maggiore (il più vicino/grande nell'immagine).
                rilevamenti, _res = self.process_all_aruco_detections()
                if not rilevamenti:
                    return

                target_marker_id, marker_corners = max(
                    rilevamenti,
                    key=lambda r: math.hypot(r[1][0][0] - r[1][1][0], r[1][0][1] - r[1][1][1])
                )

                lunghezza_lato = math.hypot(marker_corners[0][0] - marker_corners[1][0],
                                             marker_corners[0][1] - marker_corners[1][1])
                if lunghezza_lato < 15.0:
                    return

                distanza_totale_stimata = self.calculate_compensated_distance(lunghezza_lato)

                CUSCINO_SICUREZZA = 1.45
                distanza_navigazione = distanza_totale_stimata - CUSCINO_SICUREZZA

                drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                drone_ori = self.sim.getObjectOrientation(self.drone, self.sim.handle_world)
                angolo_vista_reale = drone_ori[2] + (math.pi / 2)
                quota_vincolata_tavolo = 0.8

                SOGLIA_ARRESTO_DISTANZA = 0.10
                if distanza_navigazione <= SOGLIA_ARRESTO_DISTANZA:
                    self.sim.setObjectPosition(self.drone, self.sim.handle_world,
                                                (drone_pos[0], drone_pos[1], quota_vincolata_tavolo))
                    self.aruco_mode = False
                    self.waypoint_mode = False
                    self.is_searching_aruco = False
                    self.print_msg(f"{self.C_OK}[VISION] Marker ID {target_marker_id} raggiunto. Arresto a {CUSCINO_SICUREZZA}m.{self.C_RESET}")
                    return

                distanza_navigazione = max(distanza_navigazione, 0.0)
                world_x = drone_pos[0] + (distanza_navigazione * math.cos(angolo_vista_reale))
                world_y = drone_pos[1] + (distanza_navigazione * math.sin(angolo_vista_reale))

                self.set_target_coordinates(world_x, world_y, quota_vincolata_tavolo)

                dist_residua = math.hypot(world_x - drone_pos[0], world_y - drone_pos[1])
                if dist_residua > 0.01:
                    speed_modifier = min(self.STEP_MOVE, dist_residua)
                    move_x = speed_modifier * math.cos(angolo_vista_reale)
                    move_y = speed_modifier * math.sin(angolo_vista_reale)
                    self.sim.setObjectPosition(self.drone, self.sim.handle_world,
                                                (drone_pos[0] + move_x, drone_pos[1] + move_y, quota_vincolata_tavolo))
        except Exception:
            pass

    # --- METODI DI INTERFACCIA TASTIERA DI CLASSE ---
    def on_press(self, key):
        self.key_pressed = key

    def on_release(self, key):
        self.key_pressed = None

    def process_input(self):
        if not self.key_pressed:
            return
        if self.gcs_input_active:
            self.key_pressed = None
            return

        k = self.key_pressed
        try:
            if hasattr(k, 'char') and k.char is not None:
                char = k.char.lower()
                if char not in ['t', 'q'] and not self.is_airborne:
                    return

                if char == 'w':
                    self.reset_autonomous_modes()
                    self.sim.setObjectPosition(self.drone, self.drone, (0, 0, self.STEP_MOVE))
                elif char == 's':
                    self.reset_autonomous_modes()
                    drone_pos = self.sim.getObjectPosition(self.drone, self.sim.handle_world)
                    if drone_pos[2] > 0.20:
                        nuova_alt_manuale = drone_pos[2] - self.STEP_MOVE
                        if nuova_alt_manuale < 0.20:
                            self.sim.setObjectPosition(self.drone, self.sim.handle_world, (drone_pos[0], drone_pos[1], 0.20))
                        else:
                            self.sim.setObjectPosition(self.drone, self.drone, (0, 0, -self.STEP_MOVE))
                elif char == 't':
                    self.reset_autonomous_modes()
                    self.takeoff_mode = True
                    self.key_pressed = None
                    time.sleep(0.2)
                elif char == 'g':
                    self.reset_autonomous_modes()
                    self.land_mode = True
                    self.key_pressed = None
                    time.sleep(0.2)
                elif char == 'a':
                    self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, self.STEP_YAW))
                elif char == 'd':
                    self.sim.setObjectOrientation(self.drone, self.drone, (0, 0, -self.STEP_YAW))
                elif char == 'o':
                    self.reset_autonomous_modes()
                    self.key_pressed = None
                    threading.Thread(target=self.ask_orbit_command_thread, daemon=True).start()
                    time.sleep(0.2)
                elif char == 'i':
                    self.camera_tilt += self.TILT_STEP
                    if self.camera_tilt > math.radians(50):
                        self.camera_tilt = math.radians(50)
                    self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
                elif char == 'k':
                    self.camera_tilt -= self.TILT_STEP
                    if self.camera_tilt < math.radians(-90):
                        self.camera_tilt = math.radians(-90)
                    self.sim.setFloatSignal("cameraTilt", self.camera_tilt)
                elif char == 'u':
                    if self.localization_mode or self.waypoint_aruco_mode:
                        self.print_msg(f"{self.C_WARN}[SYSTEM] Zoom bloccato: FOV costante richiesto durante localizzazione ArUco.{self.C_RESET}")
                    else:
                        self.camera_fov -= math.radians(2)
                        if self.camera_fov < math.radians(10):
                            self.camera_fov = math.radians(10)
                        self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
                elif char == 'j':
                    if self.localization_mode or self.waypoint_aruco_mode:
                        self.print_msg(f"{self.C_WARN}[SYSTEM] Zoom bloccato: FOV costante richiesto durante localizzazione ArUco.{self.C_RESET}")
                    else:
                        self.camera_fov += math.radians(2)
                        if self.camera_fov > math.radians(100):
                            self.camera_fov = math.radians(100)
                        self.sim.setObjectFloatParam(self.vision_sensor, 1004, self.camera_fov)
                elif char == 'p':
                    self.reset_autonomous_modes()
                    self.key_pressed = None
                    threading.Thread(target=self.ask_coordinates_thread, daemon=True).start()
                    time.sleep(0.2)
                elif char == 'm':
                    self.key_pressed = None
                    threading.Thread(target=self.ask_movement_command_thread, daemon=True).start()
                    time.sleep(0.2)
                elif char == 'v':
                    nuovo_stato = not self.aruco_mode
                    self.reset_autonomous_modes()
                    self.aruco_mode = nuovo_stato
                    self.key_pressed = None
                    time.sleep(0.2)
                elif char == 'l':
                    nuovo_stato = not self.localization_mode
                    self.reset_autonomous_modes()
                    self.localization_mode = nuovo_stato
                    self.key_pressed = None
                    time.sleep(0.2)
                elif char == 'z':
                    self.aruco_debug_mode = not self.aruco_debug_mode
                    if not self.aruco_debug_mode:
                        self._clear_aruco_debug_row()
                    stato = f"{self.C_OK}ON{self.C_RESET}" if self.aruco_debug_mode else f"{self.C_ERR}OFF{self.C_RESET}"
                    self.print_msg(f"{self.C_TITLE}[SYSTEM] ArUco debug mode: {stato}{self.C_RESET}")
                    self.key_pressed = None
                elif char == 'q':
                    self.running = False

            else:
                if not self.is_airborne:
                    return
                if k in (keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right):
                    if not self.movement_step_configured:
                        self.key_pressed = None
                        threading.Thread(target=self.ask_movement_command_thread, daemon=True).start()
                        time.sleep(0.2)
                        return

                if k == keyboard.Key.up:
                    self.sim.setObjectPosition(self.drone, self.drone, (0, self.STEP_MOVE, 0))
                elif k == keyboard.Key.down:
                    self.sim.setObjectPosition(self.drone, self.drone, (0, -self.STEP_MOVE, 0))
                elif k == keyboard.Key.left:
                    self.sim.setObjectPosition(self.drone, self.drone, (-self.STEP_MOVE, 0, 0))
                elif k == keyboard.Key.right:
                    self.sim.setObjectPosition(self.drone, self.drone, (self.STEP_MOVE, 0, 0))
        except Exception:
            pass

    def run(self):
        """Avvia il loop periodico a 20 Hz ancorando stabilmente la dashboard fissa."""
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        try:
            while self.running:
                self.process_input()

                # Svuota istantaneamente il buffer hardware nascosto di Windows ad ogni loop
                while msvcrt.kbhit():
                    msvcrt.getch()

                if self.takeoff_mode:
                    self.gradual_takeoff()
                elif self.land_mode:
                    self.gradual_landing()
                if self.orbit_mode:
                    self.update_orbit()
                elif self.waypoint_mode or self.aruco_mode or self.localization_mode or self.waypoint_aruco_mode:
                    self.follow_target()

                if not self.gcs_input_active:
                    line = self.get_telemetry_string()
                    if line:
                        print(f"\033[{self.ROW_TELEMETRY};1H" + self._box_row(line) + "\033[K",
                              end="", flush=True)
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            listener.stop()
            print(f"\033[{self.ROW_INPUT_AREA};1H")
            self._show_cursor()
            print(f"{self.C_TITLE}[GCS] Sessione terminata.{self.C_RESET}")


if __name__ == "__main__":
    drone_controller = Drone_Controller()
    drone_controller.run()
