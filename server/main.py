from flask import Flask, jsonify, send_from_directory, Response, render_template
from flask_cors import CORS
import cv2

#app = Flask(__name__, static_folder="../client/dist", static_url_path="/")
app = Flask(__name__)
cors = CORS(app, origins="*")

camera = cv2.VideoCapture(1)  # Capture depuis la webcam

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route("/api/users", methods=['GET'])
def users():
    return jsonify(
        {
            "users": [
                "yannette",
                "claudette",
                "danette",
                "ouga ouga"
            ]
        }
    )

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/")
def home():
    return send_from_directory(app.static_folder, "index.html")

if __name__ == "__main__":
    app.run(debug=True, port=8080)