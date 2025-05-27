import cv2
import time
import os
import re # Still useful for some validation if needed, but not primary parsing

# Importations conditionnelles pour Flask
try:
    from flask import Flask, Response, render_template_string
    flask_available = True
except ImportError:
    flask_available = False
    
try:
    import cv2.utils.logging as cv2_logging
    cv2_logging_available = True
except ImportError:
    cv2_logging_available = False
    
# --- Définition de la classe SuppressStdStreams ---
class SuppressStdStreams:
    def __init__(self):
        self.null_fd = os.open(os.devnull, os.O_RDWR)
        # Sauvegarder les descripteurs de fichiers originaux pour stdout (1) et stderr (2)
        self.save_fds = (os.dup(1), os.dup(2))

    def __enter__(self):
        # Rediriger stdout et stderr vers os.devnull
        os.dup2(self.null_fd, 1)
        os.dup2(self.null_fd, 2)

    def __exit__(self, *_):
        # Restaurer stdout et stderr
        os.dup2(self.save_fds[0], 1)
        os.dup2(self.save_fds[1], 2)
        # Fermer les descripteurs de fichiers que nous avons gérés
        os.close(self.null_fd)
        os.close(self.save_fds[0])
        os.close(self.save_fds[1])

# Variables globales pour Flask
camera_stream_obj = None
selected_device_for_stream_interactive = 0

# --- Core Camera Functions (largely unchanged) ---

def list_available_cameras():
    print("Recherche des caméras disponibles...")
    index = 0
    available_cameras_info = []

    # Tentative optionnelle de configuration du logging OpenCV (peut rester pour d'autres messages)
    original_log_level = -1
    if cv2_logging_available:
        try:
            original_log_level = cv2_logging.getLogLevel()
            # Mettre à SILENT pour une suppression maximale via le logger OpenCV
            cv2_logging.setLogLevel(cv2_logging.LOG_LEVEL_SILENT)
        except Exception:
            original_log_level = -1 # Indiquer un échec

    while index < 10: # Vérifier jusqu'à 10 périphériques
        cap = None # Initialiser cap à None pour s'assurer qu'il est défini

        # Utiliser SuppressStdStreams pour masquer les messages C++ lors de l'appel à VideoCapture
        with SuppressStdStreams():
            cap = cv2.VideoCapture(index) # Cet appel peut générer des messages d'erreur C++

        # Les vérifications et les prints suivants sont HORS du bloc 'with',
        # ils s'afficheront donc normalement.
        if cap and cap.isOpened(): # Vérifier si cap a été initialisé et est ouvert
            ret_test, _ = cap.read() # Inutile de garder frame_test ici
            if ret_test:
                width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = cap.get(cv2.CAP_PROP_FPS)
                fps_display = f"{fps:.2f} FPS" if fps > 0 else "FPS non déterminé"
                info = f"Caméra {index}: Disponible (Résolution: {int(width)}x{int(height)} @ {fps_display})"
                available_cameras_info.append({"index": index, "width": width, "height": height, "fps": fps})
                #print(f"  Trouvé: {info}")
            else:
                print(f"  Périphérique {index}: Ouvert, mais impossible de lire un frame.")
            cap.release()
        elif index == 0 and not available_cameras_info: # S'affiche seulement si l'index 0 échoue
            print(f"  Aucun périphérique trouvé à l'index {index}.")
        # Pas de 'else' pour imprimer une erreur ici si cap.isOpened() est faux pour index > 0,
        # car SuppressStdStreams devrait avoir intercepté les erreurs internes d'OpenCV.
        index += 1

    # Restaurer le niveau de log OpenCV s'il avait été changé
    if cv2_logging_available and original_log_level != -1:
        try:
            cv2_logging.setLogLevel(original_log_level)
        except Exception:
            pass # Ignorer si la restauration échoue

    if not available_cameras_info:
        print("Aucune caméra fonctionnelle n'a été détectée.")
    elif available_cameras_info:
        print("\nCaméras fonctionnelles trouvées:")
        for cam_info in available_cameras_info:
            fps_display = f"{cam_info['fps']:.2f} FPS" if cam_info['fps'] > 0 else "FPS non déterminé"
            print(f"  Index {cam_info['index']}: {int(cam_info['width'])}x{int(cam_info['height'])} @ {fps_display}")
    return available_cameras_info

def capture_images_high_res(device_index=0, base_output_path="capture.jpg", num_images=1):
    print(f"Tentative de capture de {num_images} image(s) depuis le périphérique {device_index}...")
    cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        print(f"Erreur : Impossible d'ouvrir la webcam (périphérique {device_index}).")
        return

    # Attempt to set highest possible resolution (camera will often cap this)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 9999)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 9999)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Utilisation de la résolution effective: {actual_width}x{actual_height}")

    if actual_width == 0 or actual_height == 0:
        print("Erreur : Impossible d'obtenir une résolution valide de la caméra.")
        cap.release()
        return

    time.sleep(1) # Allow camera to stabilize

    output_dir = os.path.dirname(base_output_path)
    base_filename = os.path.basename(base_output_path)
    filename_no_ext, ext = os.path.splitext(base_filename)

    if not ext: ext = ".jpg" # Default extension
    if not filename_no_ext: filename_no_ext = "capture" # Default filename base
    if not output_dir: output_dir = '.' # Default to current directory

    if output_dir != '.' and not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir, exist_ok=True)
            print(f"Création du répertoire de sortie : {output_dir}")
        except OSError as e:
            print(f"Erreur lors de la création du répertoire {output_dir}: {e}")
            cap.release()
            return

    for i in range(num_images):
        ret, frame = cap.read()
        if ret:
            # Construct filename: base_i.ext for multiple, base.ext for single
            current_output_file = os.path.join(output_dir, f"{filename_no_ext}_{i}{ext}" if num_images > 1 else f"{filename_no_ext}{ext}")
            try:
                cv2.imwrite(current_output_file, frame)
                print(f"Image {i+1}/{num_images} enregistrée sous : {os.path.abspath(current_output_file)}")
            except Exception as e:
                print(f"Erreur lors de l'enregistrement de l'image {current_output_file}: {e}")
                break # Stop if one image fails to save
            if num_images > 1 and i < num_images -1 : time.sleep(0.5) # Small delay between multiple captures
        else:
            print(f"Erreur : Impossible de capturer l'image {i+1}.")
            break
    cap.release()
    print("Caméra libérée.")

# --- Flask Streaming Functions (largely unchanged) ---
if flask_available:
    app_interactive = Flask(__name__)

    def generate_frames_flask_interactive():
        global camera_stream_obj, selected_device_for_stream_interactive
        if camera_stream_obj is None or not camera_stream_obj.isOpened():
            print(f"Initialisation du flux vidéo pour le périphérique {selected_device_for_stream_interactive}...")
            camera_stream_obj = cv2.VideoCapture(selected_device_for_stream_interactive)
            if not camera_stream_obj.isOpened():
                print(f"Erreur fatale: Impossible d'ouvrir la caméra {selected_device_for_stream_interactive} pour le streaming.")
                return # Stop generation

        while True:
            if camera_stream_obj and camera_stream_obj.isOpened():
                success, frame = camera_stream_obj.read()
                if not success:
                    print("Erreur de lecture du frame, arrêt du flux.")
                    break
                else:
                    try:
                        ret, buffer = cv2.imencode('.jpg', frame)
                        if ret:
                            frame_bytes = buffer.tobytes()
                            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                        else:
                            print("Erreur d'encodage JPEG du frame.")
                            time.sleep(0.1) # Avoid busy-looping on encode error
                    except Exception as e:
                        print(f"Exception pendant l'encodage/yield du frame: {e}")
                        break # Stop on other exceptions
            else:
                print("Flux caméra non disponible ou fermé.")
                break # Stop if camera becomes unavailable
            time.sleep(0.03) # Adjust frame rate for streaming, ~33 FPS

        if camera_stream_obj and camera_stream_obj.isOpened():
            camera_stream_obj.release()
            print("Flux caméra (générateur) libéré.")

    @app_interactive.route('/')
    def index_page_flask_interactive():
        # Simple HTML page for the stream
        return render_template_string("""
            <html><head><title>Flux Vidéo Webcam</title></head><body>
                <h1>Flux Vidéo Webcam</h1>
                <img src="{{ url_for('video_feed_flask_route_interactive') }}" width="640" height="480">
                <p>Flux depuis le périphérique {{ device_id }}. Pressez Ctrl+C dans le terminal pour arrêter le serveur et revenir au CLI.</p>
            </body></html>
        """, device_id=selected_device_for_stream_interactive)

    @app_interactive.route('/video_feed')
    def video_feed_flask_route_interactive():
        return Response(generate_frames_flask_interactive(), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_video_stream_interactive(device_index=0, port=5050, host='localhost'):
    global camera_stream_obj, selected_device_for_stream_interactive
    if not flask_available:
        print("Erreur: Flask n'est pas installé. La commande STREAM ne peut pas être exécutée.")
        print("Veuillez installer Flask avec la commande : pip install Flask")
        return

    selected_device_for_stream_interactive = device_index

    # Test camera before starting Flask server
    cam_test = cv2.VideoCapture(selected_device_for_stream_interactive)
    if not cam_test.isOpened():
        print(f"Erreur : Impossible d'ouvrir la webcam {selected_device_for_stream_interactive} pour le streaming.")
        cam_test.release()
        return
    cam_test.release() # Release test capture

    print(f"Démarrage du serveur de streaming sur http://{host}:{port} pour caméra {device_index}")
    print("Ctrl+C pour arrêter le serveur et revenir au CLI.")
    try:
        # Run Flask app
        app_interactive.run(host=host, port=port, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        print("\nServeur de streaming arrêté par l'utilisateur.")
    except Exception as e:
        print(f"Erreur durant le streaming Flask : {e}")
    finally:
        # Ensure camera is released if stream was active
        if camera_stream_obj and camera_stream_obj.isOpened():
            camera_stream_obj.release()
            print("Flux caméra (principal) libéré après arrêt du serveur.")
        print("Retour à l'invite CLI.")

# --- CLI Parsing and Execution (motors.py style) ---

def parse_command_and_execute(line):
    """Parses a command line and executes it."""
    command_full = line.strip().upper()
    parts = command_full.split()
    instruction = parts[0] if parts else ""

    print(f"CMD: {command_full}") # Echo command

    if not instruction:
        return True # Continue if empty line

    if instruction == "HELP":
        display_cli_help()

    elif instruction == "LIST_CAMERAS":
        list_available_cameras()

    elif instruction == "CAPTURE":
        # Defaults
        args_capture = {
            "OUTPUT": "captured_image.jpg", # Default output path
            "COUNT": 1,
            "DEVICE": 0
        }
        try:
            i = 1
            while i < len(parts):
                param_name = parts[i]
                if param_name == "OUTPUT" and i + 1 < len(parts):
                    args_capture["OUTPUT"] = parts[i+1] # Path can contain spaces if quoted, but simple split assumes no spaces
                                                       # For paths with spaces, user should quote and CLI might need more robust parsing
                                                       # This simple version: CAPTURE OUTPUT my_folder/img.jpg
                    i += 2
                elif param_name == "COUNT" and i + 1 < len(parts):
                    args_capture["COUNT"] = int(parts[i+1])
                    i += 2
                elif param_name == "DEVICE" and i + 1 < len(parts):
                    args_capture["DEVICE"] = int(parts[i+1])
                    i += 2
                else:
                    print(f"  Erreur: Paramètre inconnu ou valeur manquante pour '{param_name}' dans CAPTURE.")
                    return True # Continue CLI
            
            # Basic validation for count and device
            if args_capture["COUNT"] < 1:
                print("  Erreur: COUNT doit être un entier positif.")
                return True
            if args_capture["DEVICE"] < 0:
                print("  Erreur: DEVICE doit être un entier non-négatif.")
                return True

            print(f"  Paramètres Capture: Path='{args_capture['OUTPUT']}', Count={args_capture['COUNT']}, Device={args_capture['DEVICE']}")
            capture_images_high_res(
                device_index=args_capture["DEVICE"],
                base_output_path=args_capture["OUTPUT"],
                num_images=args_capture["COUNT"]
            )
        except ValueError:
            print("  Erreur: COUNT et DEVICE doivent être des entiers.")
        except Exception as e:
            print(f"  Erreur pendant la commande CAPTURE: {e}")

    elif instruction == "STREAM":
        if not flask_available:
            print("  Erreur: Flask n'est pas installé. La commande STREAM est désactivée.")
            print("  Veuillez installer Flask avec: pip install Flask")
            return True
            
        # Defaults
        args_stream = {
            "DEVICE": 0,
            "PORT": 5050,
            "HOST": "localhost"
        }
        try:
            i = 1
            while i < len(parts):
                param_name = parts[i]
                if param_name == "DEVICE" and i + 1 < len(parts):
                    args_stream["DEVICE"] = int(parts[i+1])
                    i += 2
                elif param_name == "PORT" and i + 1 < len(parts):
                    args_stream["PORT"] = int(parts[i+1])
                    i += 2
                elif param_name == "HOST" and i + 1 < len(parts):
                    args_stream["HOST"] = parts[i+1] # Host is a string
                    i += 2
                else:
                    print(f"  Erreur: Paramètre inconnu ou valeur manquante pour '{param_name}' dans STREAM.")
                    return True
            
            if args_stream["DEVICE"] < 0:
                 print("  Erreur: DEVICE doit être un entier non-négatif.")
                 return True
            if not (1024 <= args_stream["PORT"] <= 65535):
                 print("  Erreur: PORT doit être entre 1024 et 65535.")
                 return True


            print(f"  Paramètres Stream: Device={args_stream['DEVICE']}, Port={args_stream['PORT']}, Host='{args_stream['HOST']}'")
            start_video_stream_interactive(
                device_index=args_stream["DEVICE"],
                port=args_stream["PORT"],
                host=args_stream["HOST"]
            )
        except ValueError:
            print("  Erreur: DEVICE et PORT doivent être des entiers.")
        except Exception as e:
            print(f"  Erreur pendant la commande STREAM: {e}")

    elif instruction in ["EXIT", "QUIT"]:
        print("Fermeture de Webcam CLI...")
        return False # Signal to stop the main loop

    else:
        print(f"  Commande inconnue : '{instruction}'. Tapez 'HELP' pour la liste des commandes.")

    print("") # Add a blank line for better readability
    return True # Continue main loop


def display_cli_help():
    print("\n" + "=" * 48)
    print("                Camera CLI Interface")
    print("=" * 48)
    print("Available commands (case-insensitive):\n")

    print("LIST_CAMERAS")
    print("  List available webcams and their resolutions.\n")

    print("CAPTURE [OUTPUT <path>] [COUNT <n>] [DEVICE <i>]")
    print("  Capture one or more images.")
    print("  OUTPUT: file path (default: 'captured_image.jpg')")
    print("          '_N' added before extension if COUNT > 1")
    print("  COUNT:  number of images (default: 1)")
    print("  DEVICE: camera index (default: 0)\n")

    print("STREAM [DEVICE <i>] [PORT <p>] [HOST <h>]")
    print("  Start a web video stream using Flask.")
    print("  DEVICE: camera index (default: 0)")
    print("  PORT:   web server port (default: 5050)")
    print("  HOST:   host IP (default: 'localhost')\n")

    print("HELP")
    print("  Show this help message.\n")

    print("EXIT / QUIT")
    print("  Exit the application.")
    print("=" * 48 + "\n")



def main_cli():
    """Boucle principale pour l'interface en ligne de commande."""
    print("\n--- Webcam CLI (Style motors.py) ---")
    if flask_available:
        print("Flask est disponible. La commande STREAM est activée.")
    else:
        print("Flask n'est PAS installé. La commande STREAM sera désactivée.")
        print("Pour l'activer, installez Flask: pip install Flask")
    display_cli_help() # Show help at startup

    running = True
    while running:
        try:
            command_line_input = input("WebcamCLI > ").strip()
            if not command_line_input: # Skip empty input
                continue
            running = parse_command_and_execute(command_line_input)
        except KeyboardInterrupt:
            print("\nInterruption clavier. Tapez 'EXIT' ou 'QUIT' pour quitter.")
            # Optionally, could make it exit directly: running = False
        except Exception as e:
            print(f"Une erreur inattendue est survenue dans la boucle principale: {e}")
            import traceback
            traceback.print_exc() # Print full traceback for debugging

if __name__ == "__main__":
    main_cli()
    print("Programme Webcam CLI terminé.")