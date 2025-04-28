import cv2
import platform
import os
import threading

# Utiliser multiprocessing uniquement si nécessaire et avec les précautions adéquates
import multiprocessing as mp

class Webcam:
    def __init__(self, camera_index=0):
        """Initialise la webcam et démarre la capture vidéo."""
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(camera_index)

        if not self.cap.isOpened():
            raise Exception("Impossible d'ouvrir la webcam")

        self._set_max_resolution()  # Appliquer la meilleure résolution
        
        # Stocker les dimensions des images
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Capturer une image de référence
        ret, frame = self.cap.read()
        if ret:
            self.frame = frame
        else:
            self.frame = None
            
        # Flux actif
        self.running = True
        
        # Thread pour capturer les images en arrière-plan (plus léger que multiprocessing)
        self.thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.thread.start()

    def _set_max_resolution(self):
        """Détecte et applique la plus grande résolution supportée par la webcam."""
        # Résolutions courantes (du plus grand au plus petit)
        resolutions = [(3840, 2160), (2560, 1440), (1920, 1080), (1280, 720), (640, 480)]

        for width, height in resolutions:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

            # Vérifier si la résolution a bien été appliquée
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            if actual_width == width and actual_height == height:
                print(f"✅ Résolution appliquée : {width}x{height}")
                print(f"📸 Appuyer sur p pour enregister une image")
                print(f"🛑 Appuyer sur q pour quitter")
                return

        print("⚠ Impossible d'appliquer une haute résolution, utilisation de la valeur par défaut.")

    def _capture_frames(self):
        """Capture les images en arrière-plan."""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame  # Stocke l'image actuelle

    def generate_frames(self):
        """Générateur pour le streaming web."""
        while True:
            if self.frame is not None:
                # Utiliser la dernière image capturée par le thread
                frame = self.frame.copy()
                ret, buffer = cv2.imencode('.jpg', frame)
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            else:
                # Capturer une nouvelle image si aucune n'est disponible
                ret, frame = self.cap.read()
                if ret:
                    ret, buffer = cv2.imencode('.jpg', frame)
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    def show_video(self):
        """Affiche le flux vidéo et détecte les touches ('p' pour photo, 'q' pour quitter)."""
        while self.running:
            if self.frame is not None:
                cv2.imshow("Webcam", self.frame)

            key = cv2.waitKey(1) & 0xFF  # Attend 1ms et récupère la touche pressée
            if key == ord('p'):
                self.takePic("photo.jpg")  # Prendre une photo
            elif key == ord('q'):
                self.stop()  # Quitter

    def takePic(self, filename="photo.jpg"):
        """Capture et enregistre une photo sans bloquer la vidéo."""
        if self.frame is not None:
            cv2.imwrite(filename, self.frame)
            print(f"📸 Photo enregistrée : {filename}")
        else:
            # Si aucune image n'est disponible en cache, en capturer une nouvelle
            ret, frame = self.cap.read()
            if ret:
                cv2.imwrite(filename, frame)
                print(f"📸 Photo enregistrée : {filename}")
            else:
                print("❌ Erreur : aucune image disponible !")

    def stop(self):
        """Arrête la webcam proprement."""
        self.running = False
        
        # Attendre que le thread se termine
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=1)
            
        # Libérer les ressources
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            
        cv2.destroyAllWindows()
        print("📷 Webcam arrêtée.")


# Interface CLI
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Contrôle de la webcam en ligne de commande.")
    subparsers = parser.add_subparsers(dest="command", help="Commande à exécuter")

    # Sous-commande : show (affiche le flux)
    parser_show = subparsers.add_parser("show", help="Afficher le flux vidéo en direct")

    # Sous-commande : capture (enregistre une image)
    parser_capture = subparsers.add_parser("capture", help="Capturer une image")
    parser_capture.add_argument("--output", "-o", type=str, default="photo.jpg", help="Nom du fichier de sortie")

    args = parser.parse_args()

    if args.command == "show":
        cam = Webcam()
        cam.show_video()

    elif args.command == "capture":
        filename = args.output
        if not os.path.splitext(filename)[1]:  # Vérifie si une extension est présente
            filename += ".jpg"

        cam = Webcam()
        import time
        time.sleep(0.5)  # Laisse le temps à la webcam de s'allumer et au thread de capturer une frame

        cam.takePic(filename=filename)
        cam.stop()


    else:
        parser.print_help()
