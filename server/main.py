from flask import Flask, jsonify, send_from_directory, Response, render_template, request
from flask_cors import CORS
import cv2

#app = Flask(__name__, static_folder="../client/dist", static_url_path="/")
app = Flask(__name__)
cors = CORS(app, origins="*")

camera = cv2.VideoCapture(0)  # Capture depuis la webcam


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


@app.route('/take_picture', methods=['POST'])
def take_picture(): 
    picture_flag = request.json
    flag = picture_flag.get('flag')

    if not flag:
        return jsonify({"pictureFlag" : "Flag manquant"}), 400
    
    if flag == "mon super flag" :
        # Capturer une image
        ret, frame = camera.read()

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
    #camera.release()
    #cv2.destroyAllWindows()

    print(f"Flag : {picture_flag.get('flag')}")
    return jsonify({"pictureFlag" : "Flag reçu"}), 200


if __name__ == "__main__":
    app.run(debug=True, port=8080)