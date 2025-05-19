from flask import Flask, jsonify, send_from_directory, Response, request, make_response, send_file
from flask_cors import CORS
import cv2
import os
import io
import json
import datetime
import multiprocessing as mp
import time

# Importer Webcam uniquement après avoir configuré l'environnement
if __name__ == "__main__":
    # Pour éviter les problèmes avec multiprocessing sur différentes plateformes
    mp.freeze_support()
    
    # Maintenant on peut importer notre classe avec sécurité
    from control_lib import *
    
    app = Flask(__name__)
    CORS(app, origins="*")

    # Initialisation de la webcam
    camera = Webcam()
    
    # Le thread de capture est déjà démarré dans l'initialisation de Webcam
    @app.route("/video_feed")
    def video_feed():
        return Response(
            camera.generate_frames(),
            mimetype="multipart/x-mixed-replace; boundary=frame"
        )

    @app.route("/")
    def home():
        return send_from_directory(app.static_folder, "index.html")
    
    @app.route("/execute", methods=["POST"])
    def execute():
        movement_mode = request.form.get("movementMode")
        shooting_mode = request.form.get("shootingMode")
        project_name = request.form.get("projectName")
        auto_param = request.form.get("autoMovementParam")
        step_x = request.form.get("stepX")
        step_y = request.form.get("stepY")
        flag = request.form.get("flag")
        
        # Vérification des modes
        if not movement_mode or not shooting_mode:
            return jsonify({"error": "Modes incomplets"}), 400
        
        # Obtenir la date et l'heure actuelles
        now = datetime.datetime.now()

        # Formater en string (année-mois-jourheure-minute-seconde)
        timestamp = now.strftime("%m_%d_%H_%M_%S")
        
        print("\n👹 BOUTON EXECUTE :")
        
        # Traitement pour le mode "auto"
        if movement_mode == "auto":
            if not auto_param:
                return jsonify({"error": "Nombre d'images manquant"}), 400
            
            try:
                num_images = int(auto_param)
                if num_images <= 0:  # Vérifie si >0
                    return jsonify({"error": "Le nombre d'images doit être supérieur à 0"}), 400
            except ValueError:
                return jsonify({"error": "Paramètre automatique invalide (doit être un nombre)"}), 400
            
            print(f"📸 Nombre d'images à capturer : {num_images}")
            
            for i in range(num_images) : 
                camera.takePic(f"{project_name}_{timestamp}/B_{i}_Im{i}_Id{i}.png")
                time.sleep(0.2)
            
        # Traitement pour le mode "step"
        elif movement_mode == "step":
            if not step_x or not step_y:
                return jsonify({"error": "Coordonnées X et Y manquantes"}), 400
            
            try:
                x = int(step_x)
                y = int(step_y)
                print(f"📌 Déplacement pas à pas reçu - X: {x}, Y: {y}")
                
                    
            except ValueError:
                return jsonify({"error": "Coordonnées X et Y doivent être des nombres"}), 400
            
        elif movement_mode == "file":
            # Vérifier si un fichier a été uploadé
            if 'file' not in request.files:
                return jsonify({"error": "Aucun fichier trouvé"}), 400
                
            file = request.files['file']
            
            # Vérifier si le fichier est valide
            if file.filename == '':
                return jsonify({"error": "Aucun fichier sélectionné"}), 400
                
            if file and file.filename.endswith('.json'):
                # Sauvegarde du fichier
                filename = "uploaded_movement.json"
                file_path = os.path.join(app.root_path, 'uploads', filename)
                
                # Créer le dossier uploads s'il n'existe pas
                os.makedirs(os.path.dirname(file_path), exist_ok=True)
                
                file.save(file_path)
                print(f"Fichier JSON sauvegardé: {file_path}")
                
                # Votre logique pour traiter le fichier JSON
                # import json
                # with open(file_path, 'r') as f:
                #     data = json.load(f)
                # Traitement des données...
        
        return jsonify({
            "message": "Commande reçue",
            "movementMode": movement_mode,
            "shootingMode": shooting_mode
        }), 200
    
        
    @app.route("/move_step", methods=["POST"])
    def move_step():
        data = request.get_json()
        
        # Vérification des données
        if not data:
            return jsonify({"error": "Données manquantes"}), 400
        
        print("\n🚚 BOUTON DEPLACEMENT :")
        
        movement_mode = data.get("movementMode")
        step_x = data.get("stepX")
        step_y = data.get("stepY")
        flag = data.get("flag")
        
        # Validation des données
        if movement_mode != "step":
            return jsonify({"error": "Mauvais mode de mouvement"}), 400
            
        if step_x is None or step_y is None:
            return jsonify({"error": "Coordonnées manquantes"}), 400
            
        if flag != "step_movement_flag":
            return jsonify({"error": "Flag incorrect"}), 400
        
        try:
            step_x = int(step_x)
            step_y = int(step_y)
        except ValueError:
            return jsonify({"error": "Coordonnées doivent être des nombres entiers"}), 400
        
        # Ici tu peux ajouter ta logique pour déplacer le drone
        print(f"⚙️ Déplacement demandé: X={step_x}, Y={step_y}")
        
        # Exemple de réponse
        return jsonify({
            "message": "Déplacement effectué",
            "x": step_x,
            "y": step_y
        }), 200
    
        
    @app.route("/capture_step", methods=["POST"])
    def capture_step():
        data = request.get_json()
        
        # Vérification des données
        if not data:
            return jsonify({"error": "Données manquantes"}), 400
        
        print("\n🚚 BOUTON CAPTURE :")
        
        movement_mode = data.get("movementMode")
        flag = data.get("flag")
        
        # Validation des données
        if movement_mode != "step":
            return jsonify({"error": "Mauvais mode de mouvement"}), 400
            
        if flag != "step_capture_flag":
            return jsonify({"error": "Flag incorrect"}), 400
        
        print(f"📸 Capture demandée")
        
        camera.takePic("capture_step.jpg")
        
        # Exemple de réponse
        return jsonify({
            "message": "Capture effectué",
        }), 200
        
        
    @app.route('/generate_file', methods=['POST'])
    def generate_file():
        # Réception nombre images et vérification
        auto_param = request.form.get("autoMovementParam")
        try:
            auto_param = int(auto_param)  # Conversion en entier
            if auto_param <= 0:
                print("Erreur, le paramètre doit être supérieur à 0")
                return jsonify({"error": "Number lower or equal to 0"})
            # Si on arrive ici, auto_param est un entier positif valide
            generate_pos_file(auto_param)
        except ValueError:
            print("Erreur le paramètre rentré n'est pas un nombre")
            return jsonify({"error": "Not a number"})
        
        generate_pos_file(auto_param)
        
        
        print("\n⚡️ BOUTON GENERATE")
        
        dummy_data = {
            "points": [
                {"x": 0, "y": 0},
                {"x": 10, "y": 20},
                {"x": 20, "y": 40}
            ],
            "info": "Fichier de déplacement généré automatiquement"
        }
        file_content = json.dumps(dummy_data, indent=4)
        buffer = io.BytesIO()
        buffer.write(file_content.encode('utf-8'))
        buffer.seek(0)

        response = make_response(send_file(
            buffer,
            as_attachment=True,
            download_name='deplacement.json',
            mimetype='application/json'
        ))
        response.headers['Access-Control-Expose-Headers'] = 'Content-Disposition'
        return response

    # Assurer que les ressources sont libérées à la fin
    import atexit

    @atexit.register
    def cleanup():
        camera.stop()
        print("Ressources libérées")

    app.run(debug=True, port=8080)