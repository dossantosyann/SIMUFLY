import cv2
import threading
import platform

class Webcam:
    def __init__(self, camera_index=0):
        """Initialise la webcam et démarre la capture vidéo."""
        self.cap = cv2.VideoCapture(camera_index)

        if not self.cap.isOpened():
            raise Exception("Impossible d'ouvrir la webcam")

        self._set_max_resolution()  # Appliquer la meilleure résolution

        self.running = True
        self.frame = None

        # Thread pour capturer les images
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
                return

        print("⚠ Impossible d'appliquer une haute résolution, utilisation de la valeur par défaut.")


    def _capture_frames(self):
        """Capture les images en arrière-plan."""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame  # Stocke l'image actuelle

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

    def generate_frames(self):
        while True:
            success, frame = self.cap.read()
            if not success:
                break
            else:
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def takePic(self, filename="photo.jpg"):
        """Capture et enregistre une photo sans bloquer la vidéo."""
        if self.frame is not None:
            cv2.imwrite(filename, self.frame)
            print(f"📸 Photo enregistrée : {filename}")
        else:
            print("❌ Erreur : aucune image disponible !")

    def stop(self):
        """Arrête la webcam proprement."""
        self.running = False
        self.cap.release()
        cv2.destroyAllWindows()
        print("📷 Webcam arrêtée.")
