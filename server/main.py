from flask import Flask, jsonify, send_from_directory, Response, request
from flask_cors import CORS
import cv2
import os
import multiprocessing as mp

# Importer Webcam uniquement après avoir configuré l'environnement
if __name__ == "__main__":
    # Pour éviter les problèmes avec multiprocessing sur différentes plateformes
    mp.freeze_support()
    
    # Maintenant on peut importer notre classe avec sécurité
    from camlib import Webcam
    
    app = Flask(__name__)
    CORS(app, origins="*")

    # Initialisation de la webcam
    camera = Webcam()
    
    # Le thread de capture est déjà démarré dans l'initialisation de Webcam

    @app.route("/api/users", methods=["GET"])
    def users():
        return jsonify({
            "users": [
                "yannette",
                "claudette",
                "danette",
                "ouga ouga"
            ]
        })

    @app.route("/video_feed")
    def video_feed():
        return Response(
            camera.generate_frames(),
            mimetype="multipart/x-mixed-replace; boundary=frame"
        )

    @app.route("/")
    def home():
        return send_from_directory(app.static_folder, "index.html")

    @app.route("/take_picture", methods=["POST"])
    def take_picture():
        data = request.get_json()
        flag = data.get("flag")

        if not flag:
            return jsonify({"error": "Flag manquant"}), 400

        if flag == "mon super flag":
            camera.takePic("photo_webcam.jpg")
            print("📸 Photo enregistrée avec succès")
        else:
            print("❌ Flag incorrect")

        print(f"Flag reçu : {flag}")
        return jsonify({"message": "Flag reçu"}), 200

    # Assurer que les ressources sont libérées à la fin
    import atexit

    @atexit.register
    def cleanup():
        camera.stop()
        print("Ressources libérées")

    app.run(debug=True, port=8080)