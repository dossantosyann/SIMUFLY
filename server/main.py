from flask import Flask, jsonify, send_from_directory, Response, request
from flask_cors import CORS
import cv2
from camlib import Webcam  # Assure-toi que le chemin est bon

app = Flask(__name__)
CORS(app, origins="*")

# Initialisation de la webcam
camera = Webcam()

camera2 = cv2.VideoCapture(0)  # Capture depuis la webcam

def generate_frames():
    while True:
        success, frame = camera2.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

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
        generate_frames(),
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
        success, frame = camera.read()

        if success:
            cv2.imwrite("photo_webcam.jpg", frame)
            print("📸 Photo enregistrée avec succès")
        else:
            print("❌ Erreur lors de la capture d'image")

    print(f"Flag reçu : {flag}")
    return jsonify({"message": "Flag reçu"}), 200

if __name__ == "__main__":
    app.run(debug=True, port=8080)
