import curses
import re
import time
import sys
import math
import os

# Conditional imports for hardware
try:
    import gpiozero
    from gpiozero.pins.mock import MockFactory # For running on non-Pi for dev
    # To run on a Pi, comment out the next line or ensure RPi.GPIO is not installed for MockFactory to take over
    # gpiozero.Device.pin_factory = MockFactory() # REMOVE or COMMENT for ACTUAL RASPBERRY PI
    gpio_available = True
except ImportError:
    gpio_available = False
    print("WARN: gpiozero library not found. Motor control will be disabled.")

try:
    import cv2
    # Optional: OpenCV logging suppression (from camera.py)
    try:
        import cv2.utils.logging as cv2_logging
        cv2_logging_available = True
    except ImportError:
        cv2_logging_available = False
    camera_available = True
except ImportError:
    camera_available = False
    print("WARN: opencv-python library not found. Camera functionalities will be disabled.")

try:
    from flask import Flask, Response, render_template_string
    flask_available = camera_available # Flask is only useful if camera is available
except ImportError:
    flask_available = False
    if camera_available: # Only warn if camera was meant to be used with Flask
        print("WARN: Flask library not found. Camera streaming will be disabled.")

# --- Constants ---
CLI_WIDTH = 70
LEFT_INDENT_STR = ""

# --- Motor & System Parameters (from motors.py) ---
PUL_PIN_M1 = 6
DIR_PIN_M1 = 5
PUL_PIN_M2 = 27
DIR_PIN_M2 = 17

MOTOR_NATIVE_STEPS_PER_REV = 400
DRIVER_MICROSTEP_DIVISOR = 2
MM_PER_MICROSTEP = 0.01 # Adjusted from 0.5, check your actual hardware. If 0.5 is correct, revert.
if MM_PER_MICROSTEP == 0: # Ensure not zero before division
    MICROSTEPS_PER_MM = float('inf') # Or handle as an error
else:
    MICROSTEPS_PER_MM = 1.0 / MM_PER_MICROSTEP


XLIM_MM = 1000.0
YLIM_MM = 1000.0

DEFAULT_SPEED_MM_S = 50.0
HOMING_SPEED_MM_S = 50.0
MIN_PULSE_WIDTH = 0.000002  # 2 microseconds
MINIMUM_PULSE_CYCLE_DELAY = 0.0001 # 100µs

# --- GPIOZero Device Objects ---
pul_device_m1 = None
dir_device_m1 = None
pul_device_m2 = None
dir_device_m2 = None
gpio_initialized_successfully = False

# --- Hardware CLI State ---
# This will be properly initialized in simulateur_drone_curses
hardware_cli_state = {}

# --- Camera related globals (from camera.py) ---
camera_stream_obj = None # For Flask streaming
selected_device_for_stream_interactive = 0
if flask_available:
    app_interactive = Flask(__name__)
else:
    app_interactive = None # No flask app if library not present

# --- SuppressStdStreams Class (from camera.py) ---
class SuppressStdStreams:
    def __init__(self):
        self.null_fd = os.open(os.devnull, os.O_RDWR)
        self.save_fds = (os.dup(1), os.dup(2))

    def __enter__(self):
        os.dup2(self.null_fd, 1)
        os.dup2(self.null_fd, 2)

    def __exit__(self, *_):
        os.dup2(self.save_fds[0], 1)
        os.dup2(self.save_fds[1], 2)
        os.close(self.null_fd)
        os.close(self.save_fds[0])
        os.close(self.save_fds[1])

# --- Curses based display functions (largely unchanged from main.py) ---
def display_separator(stdscr, y, width=CLI_WIDTH):
    try:
        stdscr.addstr(y, 0, '-' * width)
    except curses.error: # pragma: no cover
        pass
    return y + 1

def display_title(stdscr, y, text, width=CLI_WIDTH):
    line_content = f" {text} ".center(width, '-')
    try:
        stdscr.addstr(y, 0, line_content)
    except curses.error: # pragma: no cover
        pass
    return y + 1

def display_section_header(stdscr, y, text, width=CLI_WIDTH):
    content = f"{text} :"
    try:
        stdscr.addstr(y, 0, f"{LEFT_INDENT_STR}{content}".ljust(width))
    except curses.error: # pragma: no cover
        pass
    return y + 1

def display_option(stdscr, y, text, width=CLI_WIDTH):
    try:
        stdscr.addstr(y, 0, f"{LEFT_INDENT_STR}{text}".ljust(width))
    except curses.error: # pragma: no cover
        pass
    return y + 1

def display_left_aligned_message(stdscr, y, message, width=CLI_WIDTH):
    try:
        stdscr.addstr(y, 0, " " * width) # Clear line
        stdscr.addstr(y, 0, f"{LEFT_INDENT_STR}{message}".ljust(width))
    except curses.error: # pragma: no cover
        pass
    return y + 1

def get_left_aligned_input_curses(stdscr, y, prompt_text, width=CLI_WIDTH):
    prompt_display_text = f"{LEFT_INDENT_STR}{prompt_text}"
    input_start_x = len(prompt_display_text)
    try:
        stdscr.addstr(y, 0, prompt_display_text)
        curses.echo()
        curses.curs_set(1)
        stdscr.move(y, input_start_x)
        max_input_chars = width - input_start_x - 1
        if max_input_chars < 0: max_input_chars = 0
        input_bytes = stdscr.getstr(y, input_start_x, max_input_chars)
        input_str = input_bytes.decode('utf-8', 'replace')
        curses.noecho()
        curses.curs_set(0)
        return input_str.strip()
    except curses.error: # pragma: no cover
        curses.noecho()
        curses.curs_set(0)
        return ""

def display_left_aligned_animation_char_curses(stdscr, y, char_to_display, width=CLI_WIDTH): # Inchangée
    try:
        stdscr.addstr(y, 0, f"{LEFT_INDENT_STR}{char_to_display}".ljust(width))
        stdscr.refresh()
    except curses.error: # pragma: no cover
        pass

# --- Motor Control Functions (from motors.py) ---
def setup_gpio():
    global pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2, gpio_initialized_successfully
    if not gpio_available:
        print("Skipping GPIO setup: gpiozero library not available.")
        gpio_initialized_successfully = False
        return False
    try:
        pul_device_m1 = gpiozero.OutputDevice(PUL_PIN_M1, active_high=True, initial_value=False)
        dir_device_m1 = gpiozero.OutputDevice(DIR_PIN_M1, active_high=True, initial_value=False)
        pul_device_m2 = gpiozero.OutputDevice(PUL_PIN_M2, active_high=True, initial_value=False)
        dir_device_m2 = gpiozero.OutputDevice(DIR_PIN_M2, active_high=True, initial_value=False)
        print("GPIO initialized successfully.") # Will be visible before curses takes over
        gpio_initialized_successfully = True
        return True
    except Exception as e:
        print(f"Error during GPIO initialization: {e}")
        print("Ensure you are running with necessary permissions and pins are not in use.")
        gpio_initialized_successfully = False
        return False

def cleanup_gpio():
    global pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2, gpio_initialized_successfully
    if not gpio_available or not gpio_initialized_successfully:
        return # Don't try to clean up if never setup or lib missing

    # These prints will appear after curses has ended
    if pul_device_m1: pul_device_m1.close()
    if dir_device_m1: dir_device_m1.close()
    if pul_device_m2: pul_device_m2.close()
    if dir_device_m2: dir_device_m2.close()
    print("GPIO cleaned up.")
    gpio_initialized_successfully = False


def move_motors_coordinated(steps_m1_target, steps_m2_target, pulse_cycle_delay_for_move, state):
    messages = []
    if not gpio_available or not state['gpio_initialized']:
        messages.append("Error: GPIO devices not initialized or library missing.")
        return messages

    if not all([pul_device_m1, dir_device_m1, pul_device_m2, dir_device_m2]):
        messages.append("Error: GPIO device objects are not valid.")
        return messages

    if steps_m1_target > 0: dir_device_m1.on()
    else: dir_device_m1.off()

    if steps_m2_target > 0: dir_device_m2.on()
    else: dir_device_m2.off()
    
    time.sleep(0.001) 

    abs_steps_m1 = abs(steps_m1_target)
    abs_steps_m2 = abs(steps_m2_target)
    total_iterations = max(abs_steps_m1, abs_steps_m2)

    for _ in range(total_iterations): # Use _ if i is not used
        perform_pulse_m1 = (_ < abs_steps_m1)
        perform_pulse_m2 = (_ < abs_steps_m2)

        if perform_pulse_m1: pul_device_m1.on()
        if perform_pulse_m2: pul_device_m2.on()
        time.sleep(MIN_PULSE_WIDTH)
        if perform_pulse_m1: pul_device_m1.off()
        if perform_pulse_m2: pul_device_m2.off()
        
        inter_pulse_delay = pulse_cycle_delay_for_move - MIN_PULSE_WIDTH
        if inter_pulse_delay > 0:
            time.sleep(inter_pulse_delay)
    return messages

def move_corexy(delta_x_mm, delta_y_mm, pulse_cycle_delay_for_move, state):
    motor_messages = []

    delta_x_steps_cartesian = round(delta_x_mm * MICROSTEPS_PER_MM)
    delta_y_steps_cartesian = round(delta_y_mm * MICROSTEPS_PER_MM)

    steps_m1 = delta_x_steps_cartesian + delta_y_steps_cartesian
    steps_m2 = delta_x_steps_cartesian - delta_y_steps_cartesian
    
    if steps_m1 == 0 and steps_m2 == 0:
        pass
    else:
        motor_messages = move_motors_coordinated(int(steps_m1), int(steps_m2), pulse_cycle_delay_for_move, state)

    state["current_x_mm"] += delta_x_mm
    state["current_y_mm"] += delta_y_mm
    
    state["current_x_mm"] = round(state["current_x_mm"], 3) 
    state["current_y_mm"] = round(state["current_y_mm"], 3)
    return motor_messages

# --- Camera Control Functions (from camera.py) ---
def list_available_cameras_hw(): # Renamed to avoid clash if any other list_available_cameras exists
    output_messages = ["Recherche des caméras disponibles..."]
    if not camera_available:
        output_messages.append("  Erreur: Librairie OpenCV (cv2) non disponible.")
        return output_messages, []

    index = 0
    available_cameras_info = []
    original_log_level = -1

    if cv2_logging_available:
        try:
            original_log_level = cv2_logging.getLogLevel()
            cv2_logging.setLogLevel(cv2_logging.LOG_LEVEL_SILENT)
        except Exception: # pragma: no cover
            original_log_level = -1

    while index < 10:
        cap = None
        with SuppressStdStreams(): # Suppress C++ errors from VideoCapture
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)

        if cap and cap.isOpened():
            ret_test, _ = cap.read()
            if ret_test:
                width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = cap.get(cv2.CAP_PROP_FPS)
                fps_display = f"{fps:.2f} FPS" if fps > 0 else "FPS non déterminé"
                info = f"Caméra {index}: Disponible (Résolution: {int(width)}x{int(height)} @ {fps_display})"
                available_cameras_info.append({"index": index, "width": width, "height": height, "fps": fps})
                output_messages.append(f"  Trouvé: {info}")
            else: # pragma: no cover
                output_messages.append(f"  Périphérique {index}: Ouvert, mais impossible de lire un frame.")
            cap.release()
        elif index == 0 and not available_cameras_info: # pragma: no cover
             output_messages.append(f"  Aucun périphérique trouvé à l'index {index}.")
        index += 1

    if cv2_logging_available and original_log_level != -1:
        try:
            cv2_logging.setLogLevel(original_log_level)
        except Exception: # pragma: no cover
            pass

    if not available_cameras_info:
        output_messages.append("Aucune caméra fonctionnelle n'a été détectée.")
    # else: # This part is now handled by messages appended directly
    #     output_messages.append("\nCaméras fonctionnelles trouvées:")
    #     for cam_info in available_cameras_info:
    #         fps_display = f"{cam_info['fps']:.2f} FPS" if cam_info['fps'] > 0 else "FPS non déterminé"
    #         output_messages.append(f"  Index {cam_info['index']}: {int(cam_info['width'])}x{int(cam_info['height'])} @ {fps_display}")
    return output_messages, available_cameras_info


def capture_images_high_res_hw(device_index=0, base_output_path="capture.jpg", num_images=1):
    messages = []
    if not camera_available:
        messages.append("  Erreur: Librairie OpenCV (cv2) non disponible pour CAPTURE.")
        return messages

    messages.append(f"Tentative de capture de {num_images} image(s) depuis le périphérique {device_index}...")
    cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        messages.append(f"Erreur : Impossible d'ouvrir la webcam (périphérique {device_index}).")
        return messages

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 9999)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 9999)
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    messages.append(f"Utilisation de la résolution effective: {actual_width}x{actual_height}")

    if actual_width == 0 or actual_height == 0: # pragma: no cover
        messages.append("Erreur : Impossible d'obtenir une résolution valide de la caméra.")
        cap.release()
        return messages
    time.sleep(1)

    output_dir = os.path.dirname(base_output_path) or '.'
    base_filename = os.path.basename(base_output_path)
    filename_no_ext, ext = os.path.splitext(base_filename)
    if not ext: ext = ".jpg"
    if not filename_no_ext: filename_no_ext = "capture"
    
    if output_dir != '.' and not os.path.exists(output_dir): # pragma: no cover
        try:
            os.makedirs(output_dir, exist_ok=True)
            messages.append(f"Création du répertoire de sortie : {output_dir}")
        except OSError as e:
            messages.append(f"Erreur lors de la création du répertoire {output_dir}: {e}")
            cap.release()
            return messages

    for i in range(num_images):
        ret, frame = cap.read()
        if ret:
            current_output_file = os.path.join(output_dir, f"{filename_no_ext}_{i}{ext}" if num_images > 1 else f"{filename_no_ext}{ext}")
            try:
                cv2.imwrite(current_output_file, frame)
                messages.append(f"Image {i+1}/{num_images} enregistrée sous : {os.path.abspath(current_output_file)}")
            except Exception as e: # pragma: no cover
                messages.append(f"Erreur lors de l'enregistrement de l'image {current_output_file}: {e}")
                break
            if num_images > 1 and i < num_images - 1: time.sleep(0.5)
        else: # pragma: no cover
            messages.append(f"Erreur : Impossible de capturer l'image {i+1}.")
            break
    cap.release()
    messages.append("Caméra libérée.")
    return messages

# --- Flask Streaming Functions (from camera.py, conditionally defined) ---
if flask_available and app_interactive:
    def generate_frames_flask_interactive():
        global camera_stream_obj, selected_device_for_stream_interactive # Use globals for simplicity here
        
        # Ensure any previous camera object is released before starting a new one
        if camera_stream_obj and camera_stream_obj.isOpened():
            camera_stream_obj.release()
            print("Previous camera stream object released.") # Log to console

        camera_stream_obj = cv2.VideoCapture(selected_device_for_stream_interactive, cv2.CAP_V4L2)
        if not camera_stream_obj.isOpened():
            print(f"Erreur fatale: Impossible d'ouvrir la caméra {selected_device_for_stream_interactive} pour le streaming.")
            # Yield a message or an image indicating error
            # For now, just return, which will break the client's connection.
            return

        print(f"Flux vidéo démarré pour le périphérique {selected_device_for_stream_interactive}.")
        while True:
            success, frame = camera_stream_obj.read()
            if not success:
                print("Erreur de lecture du frame, arrêt du flux.")
                break
            try:
                ret, buffer = cv2.imencode('.jpg', frame)
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                else: # pragma: no cover
                    print("Erreur d'encodage JPEG du frame.") # Log to console
                    time.sleep(0.1)
            except Exception as e: # pragma: no cover
                print(f"Exception pendant l'encodage/yield du frame: {e}") # Log to console
                break
            time.sleep(0.03) # ~33 FPS

        if camera_stream_obj and camera_stream_obj.isOpened(): # pragma: no cover
            camera_stream_obj.release()
            print("Flux caméra (générateur) libéré.") # Log to console
        camera_stream_obj = None


    @app_interactive.route('/')
    def index_page_flask_interactive():
        global selected_device_for_stream_interactive
        return render_template_string("""
            <html><head><title>Flux Vidéo Webcam</title></head><body>
                <h1>Flux Vidéo Webcam</h1>
                <img src="{{ url_for('video_feed_flask_route_interactive') }}" width="640" height="480">
                <p>Flux depuis le périphérique {{ device_id }}. Pressez Ctrl+C dans le terminal (où Flask a été lancé) pour arrêter.</p>
            </body></html>
        """, device_id=selected_device_for_stream_interactive)

    @app_interactive.route('/video_feed')
    def video_feed_flask_route_interactive():
        return Response(generate_frames_flask_interactive(), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_video_stream_interactive_hw(device_index=0, port=5050, host='localhost'):
    messages = []
    global selected_device_for_stream_interactive, app_interactive, camera_stream_obj

    if not flask_available or not app_interactive : # pragma: no cover
        messages.append("Erreur: Flask n'est pas installé ou l'application Flask n'a pas pu être initialisée. STREAM désactivé.")
        messages.append("Veuillez installer Flask avec: pip install Flask")
        return messages, False # False indicates server did not start

    if not camera_available: # pragma: no cover
        messages.append("Erreur: Librairie OpenCV (cv2) non disponible. STREAM désactivé.")
        return messages, False

    selected_device_for_stream_interactive = device_index

    # Test camera before starting Flask server
    cam_test = cv2.VideoCapture(selected_device_for_stream_interactive, cv2.CAP_V4L2)
    if not cam_test.isOpened(): # pragma: no cover
        messages.append(f"Erreur : Impossible d'ouvrir la webcam {selected_device_for_stream_interactive} pour le streaming.")
        cam_test.release()
        return messages, False
    cam_test.release()

    messages.append(f"Démarrage du serveur de streaming sur http://{host}:{port} pour caméra {device_index}")
    messages.append("Ctrl+C dans la console où CE SCRIPT tourne pour arrêter le serveur.")
    messages.append("Le CLI sera bloqué tant que le serveur Flask tourne.")
    
    # Note: app.run is blocking. This function will not return until Flask stops.
    # This has implications for the Curses UI if not handled carefully (e.g., run Flask in a thread)
    # For now, we'll let it block, and the user must Ctrl+C.
    try:
        # To make it non-blocking for curses, Flask should run in a separate thread.
        # However, for simplicity and direct porting of camera.py's behavior:
        print("\n" + "="*30)
        print(f"  Pour arrêter le STREAM et retourner au CLI principal de Drone Simulator,")
        print(f"  allez dans la FENÊTRE DU TERMINAL où vous avez lancé Drone Simulator,")
        print(f"  et pressez CTRL+C.")
        print("="*30 + "\n")

        # Release curses control temporarily if Flask is to take over the terminal with its logs
        # This is tricky. curses.endwin() then curses.initscr() might be needed.
        # For a simpler integration, we'll assume Flask logs won't break curses too much,
        # or that the user primarily interacts with the browser.
        app_interactive.run(host=host, port=port, debug=False, threaded=True, use_reloader=False)
        
        # This part will be reached after Flask server stops (e.g. Ctrl+C in terminal)
        messages.append("Serveur de streaming arrêté.") # This message might not show in Curses UI easily
        print("Serveur Flask arrêté. Retour au CLI principal...") # This will print to console
        
    except KeyboardInterrupt: # pragma: no cover
        messages.append("\nServeur de streaming arrêté par l'utilisateur (KeyboardInterrupt).")
        print("\nServeur de streaming arrêté par l'utilisateur via KeyboardInterrupt.")
    except SystemExit: # pragma: no cover
        # This can be raised by Flask's app.run on certain conditions, e.g. when reloader is on
        messages.append("Serveur de streaming terminé (SystemExit).")
        print("Serveur Flask terminé via SystemExit.")
    except Exception as e: # pragma: no cover
        messages.append(f"Erreur durant le streaming Flask : {e}")
        print(f"Erreur durant le streaming Flask: {e}")
    finally:
        if camera_stream_obj and camera_stream_obj.isOpened(): # pragma: no cover
            camera_stream_obj.release()
            camera_stream_obj = None # Reset global
            messages.append("Flux caméra (principal) libéré après arrêt du serveur.")
            print("Flux caméra (principal) libéré.")
        # No need to re-initialize curses here, assume the main loop will refresh.
    return messages, True # True indicates server attempted to start

# --- Fonctions d'interprétation des commandes ---

def interpreter_commande_capture_multi(commande_str): # Inchangée (pour mode "Automatique")
    updates = {}
    # ... (contenu original de la fonction)
    stripped_cmd_upper = commande_str.strip().upper()
    if stripped_cmd_upper == "START":
        updates["COMMAND"] = "START"
        return updates
    if stripped_cmd_upper == "BACK" or stripped_cmd_upper == "B":
        updates["COMMAND"] = "BACK"
        return updates

    pos_match = re.search(r"POS\s+(\d+)", commande_str, re.IGNORECASE)
    if pos_match:
        try:
            updates["POS"] = int(pos_match.group(1))
        except ValueError: # pragma: no cover
            pass

    n_match = re.search(r"N\s+(\d+)", commande_str, re.IGNORECASE)
    if n_match:
        try:
            updates["N"] = int(n_match.group(1))
        except ValueError: # pragma: no cover
            pass

    name_match = re.search(r"NAME\s+((?:(?!\b(?:POS|N|START|BACK)\b).)+?)(?:\s+(?:POS|N|START|BACK)|$)", commande_str, re.IGNORECASE)
    if name_match:
        name_value = name_match.group(1).strip()
        if name_value: # pragma: no branch
            updates["NAME"] = name_value
    return updates


def interpreter_commande_pas_a_pas(commande_str, drone_state): # Inchangée (simulation)
    cmd_upper = commande_str.strip().upper()
    # ... (contenu original de la fonction)
    original_cmd = commande_str.strip()
    action = None
    message = ""

    if cmd_upper == "BACK" or cmd_upper == "B":
        action = "BACK"
        return action, message

    if cmd_upper == "ABS":
        if drone_state["mode"] != "ABS": # pragma: no branch
            drone_state["mode"] = "ABS"
            message = "Mode de déplacement réglé sur ABSOLU."
        else:
            message = "Mode de déplacement déjà en ABSOLU."
        action = "STATE_UPDATED"
        return action, message

    if cmd_upper == "REL":
        if drone_state["mode"] != "REL": # pragma: no branch
            drone_state["mode"] = "REL"
            message = "Mode de déplacement réglé sur RELATIF."
        else:
            message = "Mode de déplacement déjà en RELATIF."
        action = "STATE_UPDATED"
        return action, message

    if cmd_upper == "HOME":
        drone_state["x"] = 0.0
        drone_state["y"] = 0.0
        message = f"Retour à la position HOME (0.0, 0.0)."
        action = "STATE_UPDATED"
        return action, message

    speed_match = re.match(r"SPEED\s+(\d+(?:\.\d*)?)", original_cmd, re.IGNORECASE)
    if speed_match:
        try:
            val = float(speed_match.group(1))
            if val > 0:
                drone_state["speed"] = val
                message = f"Vitesse par défaut réglée à {val}."
                action = "STATE_UPDATED"
            else: # pragma: no cover
                message = "La vitesse doit être un nombre positif."
                action = "ERROR"
        except ValueError: # pragma: no cover
            message = "Valeur de vitesse invalide."
            action = "ERROR"
        return action, message

    capture_match = re.match(r"CAPTURE\s+N\s+(\d+)", original_cmd, re.IGNORECASE)
    if capture_match:
        try:
            num_images = int(capture_match.group(1))
            if num_images > 0:
                message = f"Capture de {num_images} images à ({drone_state['x']:.1f}, {drone_state['y']:.1f}) simulée."
                action = "CAPTURE_SIMULATED"
            else: # pragma: no cover
                message = "Le nombre d'images doit être positif."
                action = "ERROR"
        except ValueError: # pragma: no cover
            message = "Nombre d'images invalide."
            action = "ERROR"
        return action, message

    move_match = re.match(r"MOVE\s+X\s+(-?\d+(?:\.\d*)?)\s+Y\s+(-?\d+(?:\.\d*)?)(?:\s+S\s+(\d+(?:\.\d*)?))?", original_cmd, re.IGNORECASE)
    if move_match:
        try:
            target_x = float(move_match.group(1))
            target_y = float(move_match.group(2))
            move_speed = drone_state["speed"]
            if move_match.group(3): # pragma: no branch
                custom_speed = float(move_match.group(3))
                if custom_speed > 0:
                    move_speed = custom_speed
                else: # pragma: no cover
                    action = "ERROR"
                    message = "La vitesse S spécifiée doit être positive."
                    return action, message
            
            old_x, old_y = drone_state["x"], drone_state["y"]
            if drone_state["mode"] == "ABS":
                drone_state["x"] = target_x
                drone_state["y"] = target_y
            else: 
                drone_state["x"] += target_x
                drone_state["y"] += target_y
            
            message = (f"Déplacement de ({old_x:.1f}, {old_y:.1f}) vers ({drone_state['x']:.1f}, {drone_state['y']:.1f}) "
                       f"à la vitesse {move_speed:.1f} (Mode: {drone_state['mode']}).")
            action = "MOVE_SIMULATED"
        except ValueError: # pragma: no cover
            message = "Coordonnées X, Y ou vitesse S invalides pour MOVE."
            action = "ERROR"
        return action, message
        
    if original_cmd: # pragma: no cover
        message = f"Commande non reconnue: '{original_cmd}'"
        action = "ERROR"
        
    return action, message

def interpreter_commande_fichier_json(commande_str, fichier_state): # Inchangée
    cmd_upper = commande_str.strip().upper()
    # ... (contenu original de la fonction)
    original_cmd = commande_str.strip()
    action = None
    message = ""

    if cmd_upper == "BACK" or cmd_upper == "B":
        action = "BACK"
        return action, message

    load_match = re.match(r"LOAD\s+(.+)", original_cmd, re.IGNORECASE) 
    if load_match:
        filename = load_match.group(1).strip()
        if not filename: # pragma: no cover
            message = "Nom de fichier manquant après LOAD."
            action = "ERROR"
        elif not filename.lower().endswith(".json"): # pragma: no cover
            message = f"Le fichier '{filename}' doit avoir une extension .json."
            action = "ERROR"
        else:
            fichier_state["loaded_filename"] = filename
            message = f"Fichier '{filename}' prêt à être utilisé."
            action = "FILE_LOADED"
        return action, message

    if cmd_upper == "START":
        if fichier_state["loaded_filename"]:
            message = f"Démarrage de la simulation avec le fichier '{fichier_state['loaded_filename']}'."
            action = "JSON_SIMULATION_START"
        else: # pragma: no cover
            message = "Aucun fichier JSON chargé. Utilisez LOAD <nom_fichier.json>."
            action = "ERROR"
        return action, message
        
    if original_cmd: # pragma: no cover
        message = f"Commande non reconnue: '{original_cmd}'"
        action = "ERROR"
        
    return action, message

def get_hardware_cli_help():
    help_messages = [
        "--- Aide Contrôle Matériel CLI ---",
        "Commandes Moteur (nécessite GPIO initialisé):",
        "  ABS                      : Mode de positionnement Absolu.",
        "  REL                      : Mode de positionnement Relatif.",
        "  HOME                     : Retourne à X=0, Y=0 (mouvement physique en ABS).",
        "  S <vitesse>              : Définit la vitesse globale (mm/s). Ex: S 100",
        "  MOVE X<val> Y<val> [S<vitesse>] : Déplace les moteurs.",
        "                             Ex: MOVE X100 Y50 S150",
        "  POS                      : Affiche la position actuelle, vitesse, mode.",
        "",
        "Commandes Caméra (nécessite OpenCV, Flask pour STREAM):",
        "  LIST_CAMERAS             : Liste les caméras disponibles.",
        "  CAPTURE [OUTPUT <path>] [COUNT <n>] [DEVICE <i>]",
        "    OUTPUT: chemin du fichier (def: captured_image.jpg)",
        "    COUNT: nombre d'images (def: 1)",
        "    DEVICE: index caméra (def: 0)",
        "    Ex: CAPTURE OUTPUT session1/img.png COUNT 5 DEVICE 1",
        "  STREAM [DEVICE <i>] [PORT <p>] [HOST <h>]",
        "    DEVICE: index caméra (def: 0)",
        "    PORT: port serveur web (def: 5050)",
        "    HOST: IP hôte (def: 'localhost')",
        "    Ex: STREAM DEVICE 1 PORT 8080",
        "    NOTE: Bloque le CLI. Arrêter avec Ctrl+C dans le terminal.",
        "",
        "Commandes Générales:",
        "  HELP                     : Affiche cette aide.",
        "  BACK / B                 : Retour au menu principal.",
        "  EXIT / QUIT              : Alias pour BACK.",
        "--- Fin Aide ---"
    ]
    return help_messages

def interpreter_commande_hardware_cli(commande_str, state):
    """Interprète les commandes pour le mode Hardware CLI."""
    output_messages = []
    action = "CONTINUE_CLI" # Default action

    cmd_full = commande_str.strip()
    cmd_upper = cmd_full.upper()
    parts = cmd_full.split()
    instruction = parts[0].upper() if parts else ""

    output_messages.append(f"CMD: {cmd_full}")

    if not instruction:
        return action, [] # Empty command, just refresh

    if instruction == "HELP":
        output_messages.extend(get_hardware_cli_help())

    # --- Motor Commands ---
    elif instruction == "ABS":
        if not state['gpio_initialized']:
            output_messages.append("  Erreur: GPIO non initialisé. Commande moteur ignorée.")
        else:
            state["absolute_mode"] = True
            output_messages.append("  Mode: Positionnement Absolu (ABS)")
    elif instruction == "REL":
        if not state['gpio_initialized']:
            output_messages.append("  Erreur: GPIO non initialisé. Commande moteur ignorée.")
        else:
            state["absolute_mode"] = False
            output_messages.append("  Mode: Positionnement Relatif (REL)")
    elif instruction == "HOME":
        if not state['gpio_initialized']:
            output_messages.append("  Erreur: GPIO non initialisé. Commande moteur ignorée.")
        else:
            output_messages.append("  Homing (HOME):")
            target_x_abs, target_y_abs = 0.0, 0.0
            dx_home, dy_home = 0.0, 0.0

            if state["absolute_mode"]:
                dx_home = target_x_abs - state["current_x_mm"]
                dy_home = target_y_abs - state["current_y_mm"]
            # In relative mode, HOME just resets coordinates without physical move here.
            # Physical move to 0,0 only happens if in ABS mode.

            if state["absolute_mode"] and (abs(dx_home) > (MM_PER_MICROSTEP / 2.0) or abs(dy_home) > (MM_PER_MICROSTEP / 2.0)):
                home_path_length_mm = math.sqrt(dx_home**2 + dy_home**2)
                output_messages.append(f"    Mouvement vers 0,0 de {state['current_x_mm']:.2f}, {state['current_y_mm']:.2f} à {HOMING_SPEED_MM_S:.2f} mm/s")
                
                if HOMING_SPEED_MM_S <= 1e-6: # pragma: no cover
                     output_messages.append(f"    Vitesse de Homing ({HOMING_SPEED_MM_S}) trop faible. Mouvement non exécuté.")
                else:
                    time_for_home_move_s = home_path_length_mm / HOMING_SPEED_MM_S
                    
                    delta_x_steps_cartesian_home = round(dx_home * MICROSTEPS_PER_MM)
                    delta_y_steps_cartesian_home = round(dy_home * MICROSTEPS_PER_MM)
                    steps_m1_home = delta_x_steps_cartesian_home + delta_y_steps_cartesian_home
                    steps_m2_home = delta_x_steps_cartesian_home - delta_y_steps_cartesian_home
                    num_iterations_home = max(abs(int(steps_m1_home)), abs(int(steps_m2_home)))

                    if num_iterations_home > 0:
                        pulse_delay_for_home_move = time_for_home_move_s / num_iterations_home
                        actual_pulse_delay_for_home_move = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_home_move)
                        motor_msgs = move_corexy(dx_home, dy_home, actual_pulse_delay_for_home_move, state)
                        output_messages.extend(motor_msgs)
                    else: # pragma: no cover
                        state["current_x_mm"] = target_x_abs # Should already be there if no steps
                        state["current_y_mm"] = target_y_abs
            else: # Already at home or in relative mode
                state["current_x_mm"] = target_x_abs
                state["current_y_mm"] = target_y_abs
                if state["absolute_mode"]:
                     output_messages.append("    Déjà à (ou très proche de) 0,0.")
                else:
                    output_messages.append("    Position logique réinitialisée à 0,0. Pas de mouvement physique en mode REL pour HOME.")
            output_messages.append(f"  Nouvelle position: X={state['current_x_mm']:.3f} mm, Y={state['current_y_mm']:.3f} mm")

    elif instruction.startswith("S") and len(instruction) > 0 and instruction != "STREAM" and instruction != "START": # Avoid clash with STREAM/START
        if not state['gpio_initialized']:
            output_messages.append("  Erreur: GPIO non initialisé. Commande moteur ignorée.")
        else:
            speed_val_str = ""
            if len(instruction) > 1 and instruction[1:].replace('.', '', 1).isdigit(): # S<val>
                 speed_val_str = instruction[1:]
            elif len(parts) > 1 and parts[1].replace('.', '', 1).isdigit(): # S <val>
                 speed_val_str = parts[1]
            
            if speed_val_str:
                try:
                    speed_val_mm_s = float(speed_val_str)
                    if speed_val_mm_s > 0:
                        state["TARGET_SPEED_MM_S"] = speed_val_mm_s
                        output_messages.append(f"  Vitesse globale définie à: {state['TARGET_SPEED_MM_S']:.2f} mm/s")
                    else: # pragma: no cover
                        output_messages.append("  La vitesse S doit être positive.")
                except ValueError: # pragma: no cover
                    output_messages.append(f"  Valeur de vitesse invalide: {speed_val_str}")
            else:
                output_messages.append(f"  Format de commande S invalide. Utilisez S<valeur> ou S <valeur>.")


    elif instruction == "MOVE":
        if not state['gpio_initialized']:
            output_messages.append("  Erreur: GPIO non initialisé. Commande moteur ignorée.")
        else:
            target_x_cmd, target_y_cmd, s_value_this_cmd = None, None, None
            try:
                for part in parts[1:]:
                    part_upper = part.upper()
                    if part_upper.startswith('X'): target_x_cmd = float(part_upper[1:])
                    elif part_upper.startswith('Y'): target_y_cmd = float(part_upper[1:])
                    elif part_upper.startswith('S'): s_value_this_cmd = float(part_upper[1:])
                
                if target_x_cmd is None and target_y_cmd is None: # pragma: no cover
                    output_messages.append("  Aucune coordonnée X ou Y spécifiée pour MOVE.")
                else: # One or both are specified
                    current_move_speed_mm_s = state["TARGET_SPEED_MM_S"]
                    if s_value_this_cmd is not None:
                        if s_value_this_cmd > 0: current_move_speed_mm_s = s_value_this_cmd
                        else: output_messages.append("  Vitesse S dans MOVE doit être positive. Utilisation de la vitesse globale.")

                    # Determine target based on mode
                    final_target_x_mm = state["current_x_mm"] if target_x_cmd is None else (target_x_cmd if state["absolute_mode"] else state["current_x_mm"] + target_x_cmd)
                    final_target_y_mm = state["current_y_mm"] if target_y_cmd is None else (target_y_cmd if state["absolute_mode"] else state["current_y_mm"] + target_y_cmd)

                    # Clamp to limits
                    actual_target_x_mm = max(0.0, min(final_target_x_mm, XLIM_MM))
                    actual_target_y_mm = max(0.0, min(final_target_y_mm, YLIM_MM))

                    if actual_target_x_mm != final_target_x_mm or actual_target_y_mm != final_target_y_mm: # pragma: no cover
                        output_messages.append(f"  Avertissement: Cible ({final_target_x_mm:.2f}, {final_target_y_mm:.2f}) hors limites.")
                        output_messages.append(f"               Sera contrainte à ({actual_target_x_mm:.2f}, {actual_target_y_mm:.2f}).")

                    delta_x_to_move = actual_target_x_mm - state["current_x_mm"]
                    delta_y_to_move = actual_target_y_mm - state["current_y_mm"]
                    path_length_mm = math.sqrt(delta_x_to_move**2 + delta_y_to_move**2)

                    if path_length_mm < (MM_PER_MICROSTEP / 2.0): # pragma: no cover
                        output_messages.append(f"  Mouvement trop petit ou déjà à la cible.")
                        state["current_x_mm"] = round(actual_target_x_mm, 3)
                        state["current_y_mm"] = round(actual_target_y_mm, 3)
                    elif current_move_speed_mm_s <= 1e-6: # pragma: no cover
                        output_messages.append(f"  Vitesse cible {current_move_speed_mm_s:.2e} mm/s trop faible. Mouvement non exécuté.")
                    else:
                        delta_x_steps = round(delta_x_to_move * MICROSTEPS_PER_MM)
                        delta_y_steps = round(delta_y_to_move * MICROSTEPS_PER_MM)
                        steps_m1 = delta_x_steps + delta_y_steps
                        steps_m2 = delta_x_steps - delta_y_steps
                        num_iterations = max(abs(int(steps_m1)), abs(int(steps_m2)))

                        if num_iterations == 0: # pragma: no cover
                             output_messages.append("  Aucun pas moteur requis pour ce mouvement.")
                             state["current_x_mm"] = round(actual_target_x_mm,3)
                             state["current_y_mm"] = round(actual_target_y_mm,3)
                        else:
                            time_for_move_s = path_length_mm / current_move_speed_mm_s
                            pulse_delay_for_this_move = time_for_move_s / num_iterations
                            actual_pulse_delay = max(MINIMUM_PULSE_CYCLE_DELAY, pulse_delay_for_this_move)
                            
                            effective_speed_mm_s = path_length_mm / (num_iterations * actual_pulse_delay) if (num_iterations * actual_pulse_delay) > 0 else 0
                            
                            output_messages.append(f"  Déplacement de dx={delta_x_to_move:.3f} mm, dy={delta_y_to_move:.3f} mm")
                            output_messages.append(f"  Vers X={actual_target_x_mm:.3f}, Y={actual_target_y_mm:.3f}")
                            output_messages.append(f"  Vitesse cible: {current_move_speed_mm_s:.2f} mm/s. Effective: {effective_speed_mm_s:.2f} mm/s.")
                            output_messages.append(f"  Délai cycle impulsion: {actual_pulse_delay*1000:.4f} ms.")
                            
                            motor_msgs = move_corexy(delta_x_to_move, delta_y_to_move, actual_pulse_delay, state)
                            output_messages.extend(motor_msgs)
                    output_messages.append(f"  Nouvelle position: X={state['current_x_mm']:.3f} mm, Y={state['current_y_mm']:.3f} mm")

            except ValueError: # pragma: no cover
                 output_messages.append("  Erreur: Valeurs X, Y, ou S invalides pour MOVE.")
            except Exception as e: # pragma: no cover
                 output_messages.append(f"  Erreur inattendue pendant MOVE: {e}")


    elif instruction == "POS":
        if not state['gpio_initialized']:
             output_messages.append("  Info: GPIO non initialisé. La position affichée est logique/simulée.")
        output_messages.append(f"  Position Actuelle: X={state['current_x_mm']:.3f} mm, Y={state['current_y_mm']:.3f} mm")
        output_messages.append(f"  Vitesse Cible: {state['TARGET_SPEED_MM_S']:.2f} mm/s")
        output_messages.append(f"  Mode: {'Absolu' if state['absolute_mode'] else 'Relatif'}")
        output_messages.append(f"  Statut GPIO: {'Initialisé' if state['gpio_initialized'] else 'Non initialisé / Erreur'}")

    # --- Camera Commands ---
    elif instruction == "LIST_CAMERAS":
        if not camera_available: # pragma: no cover
            output_messages.append("  Erreur: Librairie OpenCV (cv2) non disponible.")
        else:
            cam_list_msgs, _ = list_available_cameras_hw()
            output_messages.extend(cam_list_msgs)

    elif instruction == "CAPTURE":
        if not camera_available: # pragma: no cover
            output_messages.append("  Erreur: Librairie OpenCV (cv2) non disponible.")
        else:
            args_capture = {"OUTPUT": "captured_image.jpg", "COUNT": 1, "DEVICE": 0}
            try:
                i = 1
                while i < len(parts):
                    param_name = parts[i].upper()
                    if param_name == "OUTPUT" and i + 1 < len(parts): args_capture["OUTPUT"] = parts[i+1]; i += 2
                    elif param_name == "COUNT" and i + 1 < len(parts): args_capture["COUNT"] = int(parts[i+1]); i += 2
                    elif param_name == "DEVICE" and i + 1 < len(parts): args_capture["DEVICE"] = int(parts[i+1]); i += 2
                    else: output_messages.append(f"  Erreur: Paramètre inconnu ou valeur manquante pour '{parts[i]}'"); break
                else: # Only if loop completed without break
                    if args_capture["COUNT"] < 1: output_messages.append("  Erreur: COUNT doit être positif.")
                    elif args_capture["DEVICE"] < 0: output_messages.append("  Erreur: DEVICE doit être non-négatif.")
                    else:
                        output_messages.append(f"  Paramètres Capture: Path='{args_capture['OUTPUT']}', Count={args_capture['COUNT']}, Device={args_capture['DEVICE']}")
                        capture_msgs = capture_images_high_res_hw(
                            device_index=args_capture["DEVICE"],
                            base_output_path=args_capture["OUTPUT"],
                            num_images=args_capture["COUNT"]
                        )
                        output_messages.extend(capture_msgs)
            except ValueError: # pragma: no cover
                output_messages.append("  Erreur: COUNT et DEVICE doivent être des entiers.")
            except Exception as e: # pragma: no cover
                output_messages.append(f"  Erreur pendant CAPTURE: {e}")
                
    elif instruction == "STREAM":
        if not camera_available or not flask_available : # pragma: no cover
             output_messages.append("  Erreur: OpenCV et/ou Flask non disponibles. STREAM désactivé.")
        else:
            args_stream = {"DEVICE": 0, "PORT": 5050, "HOST": "localhost"}
            original_command_output_count = len(state["command_output"]) # To restore later
            try:
                i = 1
                while i < len(parts):
                    param_name = parts[i].upper()
                    if param_name == "DEVICE" and i+1 < len(parts): args_stream["DEVICE"] = int(parts[i+1]); i+=2
                    elif param_name == "PORT" and i+1 < len(parts): args_stream["PORT"] = int(parts[i+1]); i+=2
                    elif param_name == "HOST" and i+1 < len(parts): args_stream["HOST"] = parts[i+1]; i+=2 # Host is string
                    else: output_messages.append(f"  Erreur: Paramètre inconnu ou valeur manquante pour '{parts[i]}'"); break
                else: # Only if loop completed without break
                    if args_stream["DEVICE"] < 0: output_messages.append("  Erreur: DEVICE doit être non-négatif.")
                    elif not (1024 <= args_stream["PORT"] <= 65535): output_messages.append("  Erreur: PORT doit être entre 1024 et 65535.")
                    else:
                        output_messages.append(f"  Paramètres Stream: Device={args_stream['DEVICE']}, Port={args_stream['PORT']}, Host='{args_stream['HOST']}'")
                        # Inform user how to stop, as curses window will be 'stuck'
                        output_messages.append("  IMPORTANT: Le serveur Flask va démarrer et bloquer ce CLI.")
                        output_messages.append("  Pour arrêter le STREAM, allez dans le terminal qui exécute ce script")
                        output_messages.append("  et pressez CTRL+C. Vous reviendrez ensuite au menu principal.")
                        
                        # Temporarily clear a portion of hardware_cli_state's command_output for the stream message
                        # This is a bit of a hack to ensure the message is seen before Flask starts
                        # It relies on the main loop redrawing with this message.
                        # state["command_output"] = state["command_output"][:original_command_output_count] + output_messages[:]
                        # action = "START_STREAM_BLOCKING" # Special action to signal main loop
                        
                        # Call start_video_stream_interactive_hw which is blocking
                        # The messages it returns are for logging, won't be seen in curses UI until after it stops.
                        # Curses needs to be suspended or Flask run in a thread for seamless UI.
                        # For now, accept that curses UI is frozen.
                        curses.endwin() # Release curses control
                        print("\nCurses suspendu. Démarrage du serveur Flask...")
                        print("Messages du serveur Flask ci-dessous. Pressez CTRL+C ici pour arrêter le serveur.")
                        
                        stream_msgs, success = start_video_stream_interactive_hw(
                            device_index=args_stream["DEVICE"],
                            port=args_stream["PORT"],
                            host=args_stream["HOST"]
                        )
                        # These messages are mostly for console after Flask stops.
                        print("\nServeur Flask terminé.")
                        for msg in stream_msgs: print(msg)

                        # No Curses screen to update here, as it was ended.
                        # The main loop needs to reinitialize curses.
                        output_messages.append("Serveur de streaming terminé. Retour au menu principal.")
                        action = "BACK_TO_MAIN_MENU_AFTER_STREAM" # Special action to re-init curses
                        return action, output_messages # Return immediately
                        
            except ValueError: # pragma: no cover
                output_messages.append("  Erreur: DEVICE et PORT doivent être des entiers.")
            except Exception as e: # pragma: no cover
                output_messages.append(f"  Erreur pendant STREAM: {e}")


    # --- General Commands ---
    elif instruction in ["BACK", "B", "EXIT", "QUIT"]:
        output_messages.append("  Retour au menu principal...")
        action = "BACK_TO_MAIN_MENU"
    else:
        output_messages.append(f"  Commande inconnue : '{instruction}'. Tapez 'HELP'.")

    return action, output_messages


# --- Fonctions d'affichage des menus ---

def afficher_menu_principal_curses(stdscr, width=CLI_WIDTH):
    stdscr.clear()
    y = 0
    y = display_separator(stdscr, y, width)
    y = display_title(stdscr, y, "Drone Simulator", width)
    y = display_separator(stdscr, y, width)
    y += 1
    y = display_section_header(stdscr, y, "Sélection du mode", width) # Adjusted title
    y = display_option(stdscr, y, "0. Calibrage (Simulation)", width)
    y = display_option(stdscr, y, "1. Automatique (Simulation - TBD)", width)
    y = display_option(stdscr, y, "2. Manuel (Simulation)", width)
    y = display_option(stdscr, y, "3. Mission Fichier JSON (Simulation)", width)
    y = display_option(stdscr, y, "4. Contrôle Direct Matériel (CLI)", width) # New Option
    y = display_separator(stdscr, y, width)
    
    input_line_y = y 
    choix = get_left_aligned_input_curses(stdscr, input_line_y, "Votre choix (0-4, ou 'q' pour quitter) : ", width)
    stdscr.refresh()
    return choix, input_line_y

def afficher_sous_menu_capture_curses(stdscr, mode_deplacement_choisi, current_params, width=CLI_WIDTH): # Inchangée
    stdscr.clear()
    y = 0
    # ... (contenu original de la fonction)
    y = display_separator(stdscr, y, width)
    y = display_title(stdscr, y, "Drone Simulator", width)
    y = display_separator(stdscr, y, width)
    y += 1

    pos_str = str(current_params['POS']) if current_params['POS'] is not None else "Non défini"
    n_str = str(current_params['N']) if current_params['N'] is not None else "Non défini"
    name_str = f"'{current_params['NAME']}'" if current_params['NAME'] is not None else "Non défini"
    info_text = f"Paramètres: POS={pos_str}, N={n_str}, NAME={name_str}"
    y = display_option(stdscr, y, info_text, width)
    y += 1

    mode_text = f"Mode de déplacement choisi : {mode_deplacement_choisi}"
    y = display_option(stdscr, y, mode_text, width)
    y += 1

    y = display_section_header(stdscr, y, "Sélection du mode de capture", width)
    y = display_option(stdscr, y, "  POS <nombre> : positions de capture", width)
    y = display_option(stdscr, y, "  N <nombre>   : images par position", width)
    y = display_option(stdscr, y, "  NAME <texte> : nom du chantier", width)
    y = display_option(stdscr, y, "  START        : lance la prise de vue", width)
    y = display_option(stdscr, y, "  B / BACK     : Retour au menu principal", width)
    y = display_option(stdscr, y, "Ex: POS 10 N 5 NAME MonProjet", width)
    y = display_separator(stdscr, y, width)
    
    input_line_y = y
    commande = get_left_aligned_input_curses(stdscr, input_line_y, "Votre commande : ", width)
    stdscr.refresh()
    return commande, input_line_y

def afficher_menu_calibration_curses(stdscr, width=CLI_WIDTH): # Inchangée
    stdscr.clear()
    current_y = 0
    # ... (contenu original de la fonction)
    current_y = display_separator(stdscr, current_y, width)
    current_y = display_title(stdscr, current_y, "Drone Simulator", width)
    current_y = display_separator(stdscr, current_y, width)
    current_y += 1 
    
    current_y = display_section_header(stdscr, current_y, "Calibration en cours", width)
    animation_line_y = current_y
    
    animation_chars = ["|", "/", "-", "\\"]
    for i in range(20): # pragma: no branch
        anim_char = animation_chars[i % len(animation_chars)]
        display_left_aligned_animation_char_curses(stdscr, animation_line_y, anim_char, width)
        time.sleep(0.15)
    
    current_y = display_option(stdscr, animation_line_y, "Calibration terminée.", width) # Overwrites animation char
    current_y = display_separator(stdscr, current_y, width) # Moves to next line
    
    get_left_aligned_input_curses(stdscr, current_y, "Appuyez sur Entrée pour continuer...", width)
    stdscr.refresh()


def afficher_menu_pas_a_pas_curses(stdscr, mode_name, drone_state, width=CLI_WIDTH): # Inchangée (simulation)
    stdscr.clear()
    y = 0
    # ... (contenu original de la fonction)
    y = display_separator(stdscr, y, width)
    y = display_title(stdscr, y, "Drone Simulator - Mode Manuel (Simulation)", width)
    y = display_separator(stdscr, y, width)
    y += 1

    state_info1 = f"Position actuelle: X={drone_state['x']:.1f}, Y={drone_state['y']:.1f}"
    state_info2 = f"Vitesse par défaut: {drone_state['speed']:.1f} | Mode déplacement: {drone_state['mode']}"
    y = display_option(stdscr, y, state_info1, width)
    y = display_option(stdscr, y, state_info2, width)
    y += 1
    
    y = display_section_header(stdscr, y, "Commandes disponibles (Simulation)", width)
    y = display_option(stdscr, y, "  MOVE X <val> Y <val> [S <val>] : Déplacer le drone en XY à vitesse S", width)
    y = display_option(stdscr, y, "  ABS / REL                      : Changer mode (Absolu/Relatif)", width)
    y = display_option(stdscr, y, "  HOME                           : Retourner à X=0, Y=0", width)
    y = display_option(stdscr, y, "  SPEED <val>                    : Changer vitesse par défaut", width)
    y = display_option(stdscr, y, "  CAPTURE N <nombre>             : Capture N images", width)
    y = display_option(stdscr, y, "  B / BACK                       : Retour au menu principal", width)
    y = display_separator(stdscr, y, width)

    input_line_y = y
    commande = get_left_aligned_input_curses(stdscr, input_line_y, "Votre commande : ", width)
    stdscr.refresh()
    return commande, input_line_y

def afficher_menu_fichier_json_curses(stdscr, fichier_state, width=CLI_WIDTH): # Inchangée
    stdscr.clear()
    y = 0
    # ... (contenu original de la fonction)
    y = display_separator(stdscr, y, width)
    y = display_title(stdscr, y, "Drone Simulator - Mode Fichier JSON (Simulation)", width)
    y = display_separator(stdscr, y, width)
    y += 1

    fichier_charge_text = f"Fichier chargé : {fichier_state['loaded_filename'] if fichier_state['loaded_filename'] else 'Aucun'}"
    y = display_option(stdscr, y, fichier_charge_text, width)
    y += 1
    
    y = display_section_header(stdscr, y, "Commandes disponibles", width)
    y = display_option(stdscr, y, "  LOAD <nom_fichier.json>        : Charger un fichier de mission", width)
    y = display_option(stdscr, y, "  START                          : Démarrer la mission du fichier", width)
    y = display_option(stdscr, y, "  B / BACK                       : Retour au menu principal", width)
    y = display_separator(stdscr, y, width)

    input_line_y = y
    commande = get_left_aligned_input_curses(stdscr, input_line_y, "Votre commande : ", width)
    stdscr.refresh()
    return commande, input_line_y

def afficher_menu_hardware_cli_curses(stdscr, state, width=CLI_WIDTH):
    stdscr.clear()
    max_y, max_x = stdscr.getmaxyx() # Use max_x for width calculations
    current_y = 0

    # Header Section
    title = "Contrôle Direct Matériel (CLI)"
    current_y = display_title(stdscr, current_y, title, max_x -1) # Use max_x-1 for full width
    
    # Status Section
    status_lines = [
        f" Position: X={state['current_x_mm']:.3f}mm Y={state['current_y_mm']:.3f}mm | Mode: {'ABS' if state['absolute_mode'] else 'REL'}",
        f" Vitesse Cible: {state['TARGET_SPEED_MM_S']:.2f} mm/s | GPIO: {'OK' if state['gpio_initialized'] else 'NON OK/Désactivé'}",
        f" Caméra: {'OK' if camera_available else 'NON OK'} | Flask: {'OK' if flask_available else 'NON OK'}"
    ]
    for line in status_lines:
        if current_y < max_y -1 : # Ensure space for separator
            stdscr.addstr(current_y, 0, line[:max_x-1])
            current_y +=1
    if current_y < max_y -1:
        stdscr.addstr(current_y, 0, "-" * (max_x -1) )
        current_y +=1
    
    # Command Output Section
    output_start_y = current_y
    # Reserve 2 lines: 1 for separator, 1 for input prompt
    available_lines_for_output = max_y - output_start_y - 2 
    if available_lines_for_output < 0 : available_lines_for_output = 0 # Handle small terminal

    # Display last N lines of command_output
    num_output_lines_to_show = available_lines_for_output
    start_index = 0
    if len(state["command_output"]) > num_output_lines_to_show:
        start_index = len(state["command_output"]) - num_output_lines_to_show
    
    for i, line_out in enumerate(state["command_output"][start_index:]):
        if output_start_y + i < max_y - 2: 
            # Ensure line is not too long and properly padded/truncated
            display_text = (LEFT_INDENT_STR + line_out)[:max_x-1]
            stdscr.addstr(output_start_y + i, 0, display_text.ljust(max_x-1))
    
    # Separator before input
    input_line_y_sep = max_y - 2
    if input_line_y_sep >= output_start_y + len(state["command_output"][start_index:]): # Only if not overwriting output
         if input_line_y_sep > 0 : stdscr.addstr(input_line_y_sep, 0, "-" * (max_x-1))
    elif input_line_y_sep > 0 : # Overwrite last line of output if necessary
         stdscr.addstr(input_line_y_sep, 0, "-" * (max_x-1))


    # Input Prompt
    input_line_y = max_y - 1
    prompt = "HardwareCLI > "
    if input_line_y > 0: # Check if terminal is tall enough
        # Using get_left_aligned_input_curses for consistency and features
        # Temporarily clear the state's command_output for the input function,
        # as it might get polluted by its own drawing if not careful.
        # Or, better, make get_left_aligned_input_curses more robust.
        # For now, directly use stdscr.getstr with echo management.
        stdscr.addstr(input_line_y, 0, prompt)
        stdscr.refresh() # Refresh to show prompt before input
        curses.echo()
        curses.curs_set(1)
        stdscr.move(input_line_y, len(prompt))
        
        max_input_len = max_x - len(prompt) -1 
        if max_input_len <0: max_input_len = 0

        cmd_bytes = stdscr.getstr(input_line_y, len(prompt), max_input_len)
        cmd_str = cmd_bytes.decode('utf-8', 'replace').strip()
        curses.noecho()
        curses.curs_set(0)
    else: # pragma: no cover
        cmd_str = "" # Terminal too small

    stdscr.refresh()
    return cmd_str, input_line_y # Return command and where input happened


# --- Logique principale du simulateur ---

def simulateur_drone_curses(stdscr):
    global gpio_initialized_successfully # To know the status from setup_gpio
    global hardware_cli_state # Allow modification
    
    # Initialize hardware_cli_state here, once.
    hardware_cli_state = {
        "current_x_mm": 0.0,
        "current_y_mm": 0.0,
        "TARGET_SPEED_MM_S": DEFAULT_SPEED_MM_S,
        "absolute_mode": True,
        "gpio_initialized": gpio_initialized_successfully, # Set by setup_gpio()
        "command_output": ["Bienvenue au CLI Matériel. Tapez 'HELP' pour les commandes."], # History of commands/outputs
        "MAX_OUTPUT_LINES": 100 # Max history
    }


    curses.curs_set(0)
    curses.noecho()
    stdscr.keypad(True)

    parametres_capture_auto = {"POS": None, "N": None, "NAME": None}
    message_display_duration = 1.5

    main_loop_active = True
    while main_loop_active:
        # Re-check/update GPIO status in state if it could change (though it usually doesn't after init)
        hardware_cli_state["gpio_initialized"] = gpio_initialized_successfully

        choix_principal, main_menu_input_line_y = afficher_menu_principal_curses(stdscr, CLI_WIDTH)
        main_menu_message_y = main_menu_input_line_y + 1

        if choix_principal == '0':
            afficher_menu_calibration_curses(stdscr, CLI_WIDTH)

        elif choix_principal == '1':
            mode_actuel = "Automatique" 
            # ... (contenu original de la boucle du mode Automatique)
            while True:
                commande_brute, sub_menu_input_line_y = afficher_sous_menu_capture_curses(
                    stdscr, mode_actuel, parametres_capture_auto, CLI_WIDTH
                )
                message_y_pos_submenu = sub_menu_input_line_y + 1
                parsed_updates = interpreter_commande_capture_multi(commande_brute)
                messages_to_show = []

                if "POS" in parsed_updates:
                    parametres_capture_auto["POS"] = parsed_updates["POS"]
                    messages_to_show.append(f"Nombre de positions défini à : {parametres_capture_auto['POS']}")
                if "N" in parsed_updates:
                    parametres_capture_auto["N"] = parsed_updates["N"]
                    messages_to_show.append(f"Nombre d'images/pos défini à : {parametres_capture_auto['N']}")
                if "NAME" in parsed_updates:
                    parametres_capture_auto["NAME"] = parsed_updates["NAME"]
                    messages_to_show.append(f"Nom du chantier défini à : '{parametres_capture_auto['NAME']}'")

                if messages_to_show:
                    current_msg_line = message_y_pos_submenu
                    for msg_line in messages_to_show: # pragma: no branch
                        display_left_aligned_message(stdscr, current_msg_line, msg_line, CLI_WIDTH)
                        current_msg_line +=1
                    stdscr.refresh()
                    time.sleep(message_display_duration)

                commande_speciale = parsed_updates.get("COMMAND")
                if commande_speciale == "START":
                    if all(p is not None for p in parametres_capture_auto.values()): # pragma: no branch
                        stdscr.clear()
                        y = 0 # Start from top
                        y = display_left_aligned_message(stdscr, y+1, f"Lancement: mode '{mode_actuel}'", CLI_WIDTH)
                        y = display_left_aligned_message(stdscr, y, f"  Positions : {parametres_capture_auto['POS']}", CLI_WIDTH)
                        y = display_left_aligned_message(stdscr, y, f"  Images/pos: {parametres_capture_auto['N']}", CLI_WIDTH)
                        y = display_left_aligned_message(stdscr, y, f"  Chantier  : '{parametres_capture_auto['NAME']}'", CLI_WIDTH)
                        y += 1
                        y = display_left_aligned_message(stdscr, y, "Simulation de la prise de vue terminée.", CLI_WIDTH)
                        y = display_separator(stdscr, y, CLI_WIDTH)
                        get_left_aligned_input_curses(stdscr, y, "Appuyez sur Entrée pour retourner...", CLI_WIDTH) # Shorter prompt
                        break 
                    else: # pragma: no cover
                        display_left_aligned_message(stdscr, message_y_pos_submenu, "Veuillez définir POS, N, et NAME avant START.", CLI_WIDTH)
                        stdscr.refresh()
                        time.sleep(message_display_duration)
                elif commande_speciale == "BACK":
                    break 
                elif not parsed_updates and commande_brute.strip(): # pragma: no cover
                    display_left_aligned_message(stdscr, message_y_pos_submenu, f"Commande non reconnue : '{commande_brute}'.", CLI_WIDTH)
                    stdscr.refresh()
                    time.sleep(message_display_duration)


        elif choix_principal == '2':
            drone_state_pas_a_pas = { "x": 0.0, "y": 0.0, "speed": 10.0, "mode": "ABS" }
            # ... (contenu original de la boucle du mode Manuel/Pas-à-pas)
            while True:
                commande_brute, sub_menu_input_line_y = afficher_menu_pas_a_pas_curses(
                    stdscr, "Manuel (Simulation)", drone_state_pas_a_pas, CLI_WIDTH
                )
                message_y_pos_submenu = sub_menu_input_line_y + 1
                action, message = interpreter_commande_pas_a_pas(commande_brute, drone_state_pas_a_pas)

                if message:
                    display_left_aligned_message(stdscr, message_y_pos_submenu, message, CLI_WIDTH)
                    stdscr.refresh()
                    time.sleep(message_display_duration if action != "ERROR" else message_display_duration + 0.5) # pragma: no cover
                
                if action == "BACK":
                    break

        elif choix_principal == '3':
            fichier_json_state = {"loaded_filename": None}
            # ... (contenu original de la boucle du mode Fichier JSON)
            while True:
                commande_brute, sub_menu_input_line_y = afficher_menu_fichier_json_curses(
                    stdscr, fichier_json_state, CLI_WIDTH
                )
                message_y_pos_submenu = sub_menu_input_line_y + 1
                action, message = interpreter_commande_fichier_json(commande_brute, fichier_json_state)

                if message: # pragma: no branch
                    display_left_aligned_message(stdscr, message_y_pos_submenu, message, CLI_WIDTH)
                    stdscr.refresh()
                    time.sleep(message_display_duration if action != "ERROR" else message_display_duration + 0.5) # pragma: no cover

                if action == "BACK":
                    break
                elif action == "JSON_SIMULATION_START": # pragma: no branch
                    stdscr.clear()
                    y = 0
                    y = display_left_aligned_message(stdscr, y+1, f"Lancement: Mode Fichier JSON", CLI_WIDTH)
                    y = display_left_aligned_message(stdscr, y, f"  Fichier   : '{fichier_json_state['loaded_filename']}'", CLI_WIDTH)
                    y += 1
                    y = display_left_aligned_message(stdscr, y, "Simulation de la mission à partir du fichier terminée.", CLI_WIDTH)
                    y = display_separator(stdscr, y, CLI_WIDTH)
                    get_left_aligned_input_curses(stdscr, y, "Appuyez sur Entrée pour retourner...", CLI_WIDTH)
                    break 
        
        elif choix_principal == '4': # Hardware CLI Mode
            hardware_cli_active = True
            # Reset or ensure command output starts fresh or with a welcome
            if not hardware_cli_state["command_output"] or hardware_cli_state["command_output"][-1] != "Bienvenue au CLI Matériel. Tapez 'HELP' pour les commandes.":
                 hardware_cli_state["command_output"] = ["Bienvenue au CLI Matériel. Tapez 'HELP' pour les commandes."]


            while hardware_cli_active:
                # Ensure curses is properly initialized for this loop iteration
                # This is especially important if returning from Flask stream
                stdscr.keypad(True) # Ensure keypad is on
                curses.noecho()    # Ensure echo is off initially
                curses.curs_set(0) # Hide cursor initially

                commande_hw_cli, _ = afficher_menu_hardware_cli_curses(stdscr, hardware_cli_state, CLI_WIDTH)
                
                action_hw_cli, messages_hw_cli = interpreter_commande_hardware_cli(commande_hw_cli, hardware_cli_state)
                
                hardware_cli_state["command_output"].extend(messages_hw_cli)
                # Limit history size
                if len(hardware_cli_state["command_output"]) > hardware_cli_state["MAX_OUTPUT_LINES"]: # pragma: no cover
                    hardware_cli_state["command_output"] = hardware_cli_state["command_output"][-hardware_cli_state["MAX_OUTPUT_LINES"]:]

                if action_hw_cli == "BACK_TO_MAIN_MENU":
                    hardware_cli_active = False
                elif action_hw_cli == "QUIT_APP": # Should not be triggered by current interpreter
                    hardware_cli_active = False # pragma: no cover
                    main_loop_active = False    # pragma: no cover
                elif action_hw_cli == "BACK_TO_MAIN_MENU_AFTER_STREAM":
                    hardware_cli_active = False
                    # Curses was ended by the stream function, need to re-init for the main menu
                    # The wrapper should handle this, but let's explicitly refresh
                    # No, the main loop will call afficher_menu_principal_curses which will clear and redraw.
                    # Just break from this sub-loop.
                    # curses.doupdate() # Force an update if needed, though next menu display should handle it
                    # Re-initialize curses screen if it was ended by Flask.
                    # This is tricky with curses.wrapper. The wrapper handles initscr and endwin.
                    # For now, we assume the wrapper re-establishes the screen.
                    # The user will see console output from Flask, then back to Curses.
                    # A small delay might be good for the user to read console messages.
                    time.sleep(0.5) # Allow user to see any post-Flask console messages.
                    # stdscr = curses.initscr() # This would be needed if not using wrapper or if wrapper doesn't restore fully
                    # curses.noecho()
                    # curses.cbreak() # or raw()
                    # stdscr.keypad(True)
                    # curses.curs_set(0)
                    # stdscr.clear() # Clear screen before showing main menu again
                    # stdscr.refresh()
                    break # Break hardware_cli_active loop to go to main menu

                # Refresh the display with new output (handled by next call to afficher_menu_hardware_cli_curses)
                stdscr.refresh() # ensure screen is up-to-date

        elif choix_principal.lower() == 'q':
            stdscr.clear()
            y = 0
            # ... (contenu original de la fonction de sortie)
            y = display_left_aligned_message(stdscr, y+1, "Drone Simulator arrêté. Au revoir !", CLI_WIDTH) # y+1 to avoid top line
            y = display_separator(stdscr, y, CLI_WIDTH)
            stdscr.refresh()
            time.sleep(1)
            main_loop_active = False # Use loop control variable
        
        else: # Choix invalide au menu principal
            display_left_aligned_message(stdscr, main_menu_message_y, "Choix invalide. Veuillez réessayer.", CLI_WIDTH)
            stdscr.refresh()
            time.sleep(message_display_duration)

if __name__ == "__main__":
    # --- Initial GPIO Setup ---
    # This print happens before Curses takes over
    print("Initialisation du simulateur de drone...")
    if gpio_available:
        print("Tentative d'initialisation GPIO...")
        if setup_gpio(): # Sets gpio_initialized_successfully global
            print("Configuration GPIO réussie.")
        else: # pragma: no cover
            print("Échec de la configuration GPIO. Les fonctionnalités moteur matériel seront désactivées.")
    else: # pragma: no cover
        print("gpiozero non disponible. Les fonctionnalités moteur matériel sont désactivées.")

    if camera_available:
        print("Librairie OpenCV (cv2) trouvée.")
        if flask_available and app_interactive:
            print("Librairie Flask trouvée. Streaming activé.")
        else: # pragma: no cover
            print("Flask non trouvée ou erreur init app. Streaming désactivé.")
    else: # pragma: no cover
        print("Librairie OpenCV (cv2) NON trouvée. Fonctionnalités caméra désactivées.")

    print("Lancement de l'interface Curses...")
    time.sleep(0.1) # Brief pause so user can see pre-curses messages

    try:
        curses.wrapper(simulateur_drone_curses)
    except KeyboardInterrupt: # pragma: no cover
        # Cleanup is now handled in finally block after wrapper exits
        print("Interruption clavier détectée (hors wrapper Curses). Fermeture.")
    except curses.error as e: # pragma: no cover
        print(f"Une erreur Curses (hors wrapper) est survenue: {e}")
    finally:
        # --- Final GPIO Cleanup ---
        if gpio_available and gpio_initialized_successfully: # only cleanup if successfully initialized
            print("Nettoyage GPIO avant de quitter...")
            cleanup_gpio()
        print("Programme Drone Simulator terminé.")