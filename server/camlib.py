import cv2
import threading

class Webcam:
    def __init__(self, camera_index=0):
        """Initialise la webcam et lance le thread pour l'affichage vidéo."""
        self.cap = cv2.VideoCapture(camera_index)
        self.running = True

        if not self.cap.isOpened():
            raise Exception("Impossible d'ouvrir la webcam")

        # Démarrer le thread pour afficher le flux vidéo
        self.thread = threading.Thread(target=self._show_video, daemon=True)
        self.thread.start()

    def _show_video(self):
        """Affiche le flux vidéo en temps réel."""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("Erreur de lecture vidéo")
                break

            cv2.imshow("Webcam", frame)
            
            # Fermer si 'q' est pressé
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()

    def takePic(self, filename="photo.jpg"):
        """Capture et enregistre une photo."""
        ret, frame = self.cap.read()
        if ret:
            cv2.imwrite(filename, frame)
            print(f"Photo enregistrée : {filename}")
        else:
            print("Erreur lors de la capture d'image")

    def stop(self):
        """Arrête la webcam proprement."""
        self.running = False
        self.cap.release()
        cv2.destroyAllWindows()

# Exemple d'utilisation
if __name__ == "__main__":
    cam = Webcam()
    input("Appuyez sur Entrée pour prendre une photo...")  # Simulation de prise de photo
    cam.takePic("test_photo.jpg")
    input("Appuyez sur Entrée pour quitter...")
    cam.stop()
