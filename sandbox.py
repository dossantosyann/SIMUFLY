import cv2
import time

from server.camlib import *

#cam = Webcam()
#cam.takePic("ma_photo.jpg")  # Capture une image
#cam.stop()  # Ferme la webcam proprement


 # Initialiser la webcam (0 pour la webcam par défaut)
cap = cv2.VideoCapture(0)

"""
# Pour régler le gain et le temps d'exposition
# Les propriétés exactes peuvent varier selon votre webcam
cap.set(cv2.CAP_PROP_GAIN, 50)  # Régler le gain (valeur entre 0-100)
cap.set(cv2.CAP_PROP_EXPOSURE, -6)  # Régler l'exposition (souvent en valeur log)

# Vérifier si la webcam est ouverte correctement
if not cap.isOpened():
    print("Erreur lors de l'ouverture de la webcam")
    exit()

# Attendre que la webcam s'initialise
time.sleep(2)


# Capturer une image
ret, frame = cap.read()

# Vérifier si l'image a été capturée
if ret:
    # Enregistrer l'image
    cv2.imwrite('photo_webcam.jpg', frame)
    print("Photo enregistrée avec succès")
    
    # Afficher l'image (optionnel)
    cv2.imshow('Photo', frame)
    cv2.waitKey(0)
else:
    print("Erreur lors de la capture d'image")

# Libérer la webcam
cap.release()
cv2.destroyAllWindows() """