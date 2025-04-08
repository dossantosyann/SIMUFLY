from server.camlib import *

cam = Webcam()

# Lancer l'affichage (doit être exécuté dans le thread principal)
cam.show_video()

# Fermer proprement
cam.stop()
