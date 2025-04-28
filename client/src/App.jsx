import { useState } from 'react'
import heigLogo from './assets/heigLogo.svg'
import './App.css'
import axios from 'axios';

function App() {
  const [movementMode, setMovementMode] = useState("");
  const [shootingMode, setShootingMode] = useState("");
  const [autoMovementParam, setAutoMovementParam] = useState("");
  const [selectedFile, setSelectedFile] = useState(null); // Nouvel état pour le fichier
  const [fileName, setFileName] = useState("Aucun fichier sélectionné"); // Pour afficher le nom du fichier
  const [stepX, setStepX] = useState("");
  const [stepY, setStepY] = useState("");

  const handleMovementSelection = (mode) => {
    setMovementMode(mode);
  };

  const handleShootingSelection = (mode) => {
    setShootingMode(mode);
  };

  const handleFileChange = (event) => {
    const file = event.target.files[0];
    if (file && file.name.endsWith('.json')) {
      setSelectedFile(file);
      setFileName(file.name);
    } else {
      alert("Veuillez sélectionner un fichier JSON valide.");
      event.target.value = null;
      setSelectedFile(null);
      setFileName("Aucun fichier sélectionné");
    }
  };

  const handleExecute = () => {
    // Logique pour envoyer les modes sélectionnés au backend
    console.log("Exécution avec mode de déplacement:", movementMode, "et mode de prise de vue:", shootingMode);

    // Créer un objet FormData pour l'envoi de fichier
    const formData = new FormData();
    formData.append('movementMode', movementMode);
    formData.append('shootingMode', shootingMode);

    // Ajouter les paramètres spécifiques selon le mode
    if (movementMode === "auto") {
      formData.append('autoMovementParam', autoMovementParam);
    } else if (movementMode === "file" && selectedFile) {
      formData.append('file', selectedFile);
    } else if (movementMode === "step") {
      formData.append('stepX', stepX);
      formData.append('stepY', stepY);
    }

    // Configuration pour l'envoi du fichier
    const config = {
      headers: {
        'Content-Type': 'multipart/form-data'
      }
    };

    // Envoi au serveur
    axios.post('http://localhost:8080/execute', formData, config)
      .then(response => {
        console.log("Réponse du serveur:", response.data);
        // Traitement de la réponse
      })
      .catch(error => {
        console.error("Erreur lors de l'exécution:", error);
      });
  };

  const handleMove = () => {
    // Vérifie que les valeurs sont valides
    if (!stepX || !stepY) return;

    // Crée un objet avec les données à envoyer
    const moveData = {
      movementMode: "step",
      stepX: parseInt(stepX),
      stepY: parseInt(stepY),
      flag: "step_movement_flag" // Flag pour identifier le déplacement pas à pas
    };

    // Envoie les données au serveur
    axios.post('http://localhost:8080/move_step', moveData)
      .then(response => {
        console.log("Réponse du serveur:", response.data);
      })
      .catch(error => {
        console.error("Erreur lors du déplacement:", error);
      });
  };

  const handleCapture = () => {
    // Vérifie que les valeurs sont valides
    if (!stepX || !stepY) return;

    // Crée un objet avec les données à envoyer
    const moveData = {
      movementMode: "step",
      flag: "step_capture_flag" // Flag pour identifier le déplacement pas à pas
    };

    // Envoie les données au serveur
    axios.post('http://localhost:8080/capture_step', moveData)
      .then(response => {
        console.log("Réponse du serveur:", response.data);
      })
      .catch(error => {
        console.error("Erreur lors de la capture:", error);
      });
  };

  // Vérifiez si les deux modes sont sélectionnés ET les paramètres nécessaires sont fournis
  const canExecute = () => {
    if (movementMode === "" || shootingMode === "") return false;
    if (movementMode === "auto" && autoMovementParam === "") return false;
    if (movementMode === "file" && !selectedFile) return false;
    return true;
  };

  return (
    <>
      <h1>Drone Simulator</h1>

      {/* Flux vidéo toujours visible */}
      <div className="video-container">
        <h2>Flux vidéo</h2>
        <img src="http://localhost:8080/video_feed" alt="Flux vidéo en direct" />
      </div>

      {/* Section choix du mode de déplacement */}
      <div className="mode-title">
        <h2>Mode de déplacement :</h2>
      </div>
      <div className="button-row fade-in-up">
        <button
          className={`base-button ${movementMode === "auto" ? "selected-button" : ""}`}
          onClick={() => handleMovementSelection("auto")}
        >
          Automatique
        </button>

        <button
          className={`base-button ${movementMode === "file" ? "selected-button" : ""}`}
          onClick={() => handleMovementSelection("file")}
        >
          À partir d'un fichier
        </button>

        <button
          className={`base-button ${movementMode === "step" ? "selected-button" : ""}`}
          onClick={() => handleMovementSelection("step")}
        >
          Pas à pas
        </button>
      </div>

      {/* Zone de paramètres pour le mode automatique */}
      {movementMode === "auto" && (
        <div className="parameter-zone fade-in-up">
          <label htmlFor="autoParam">Nombre d'images à capturer :</label>
          <input
            type="text"  // On utilise "text" pour mieux contrôler l'input
            id="autoParam"
            value={autoMovementParam}
            onChange={(e) => {
              const val = e.target.value;
              // Autorise uniquement les chiffres et supprime tout le reste
              const filteredVal = val.replace(/[^0-9]/g, '');
              // Met à jour l'état uniquement si c'est un nombre valide >0 ou vide
              if (filteredVal === "" || parseInt(filteredVal) > 0) {
                setAutoMovementParam(filteredVal);
              }
            }}
            placeholder="Entrez un nombre"
          />
        </div>
      )}

      {/* Zone pour upload de fichier JSON (mode "file") */}
      {movementMode === "file" && (
        <div className="parameter-zone file-upload-zone fade-in-up">
          <label htmlFor="fileUpload">Chargez un fichier JSON :</label>
          <div className="file-input-container">
            <button className="file-button" onClick={() => document.getElementById('fileUpload').click()}>
              Parcourir...
            </button>
            <span className="file-name">{fileName}</span>
            <input
              type="file"
              id="fileUpload"
              accept=".json"
              onChange={handleFileChange}
              style={{ display: 'none' }}
            />
          </div>
        </div>
      )}

      {movementMode === "step" && (
        <div className="parameter-zone fade-in-up">
          <label>Déplacement (X, Y) :</label>
          <div style={{ display: 'flex', gap: '10px' }}>
            <input
              type="text"
              value={stepX}
              onChange={(e) => {
                let val = e.target.value.replace(/[^0-9-]/g, '');
                // Supprime les '-' en double et ceux qui ne sont pas en première position
                if (val.includes('-')) {
                  val = '-' + val.replace(/-/g, '');
                }
                setStepX(val); // ou setStepY
              }}
              placeholder="X"
            />
            <input
              type="text"
              value={stepY}
              onChange={(e) => {
                let val = e.target.value.replace(/[^0-9-]/g, '');
                // Supprime les '-' en double et ceux qui ne sont pas en première position
                if (val.includes('-')) {
                  val = '-' + val.replace(/-/g, '');
                }
                setStepY(val); // ou setStepY
              }}
              placeholder="Y"
            />
            <button
              onClick={handleMove}
              disabled={!stepX || !stepY}
              className="move-button" // Même style que le bouton JSON
              style={{
                marginLeft: "10px",
                padding: "8px 16px",
                backgroundColor: !stepX || !stepY ? "#cccccc" : "#4CAF50" // Gris si désactivé
              }}
            >
              Déplacer
            </button>
            <button
              onClick={handleCapture}
              disabled={!stepX || !stepY}
              className="capture-button" // Même style que le bouton JSON
              style={{
                marginLeft: "10px",
                padding: "8px 16px",
                backgroundColor: !stepX || !stepY ? "#cccccc" : "#4CAF50" // Gris si désactivé
              }}
            >
              Capturer
            </button>
          </div>
        </div>
      )}

      {/* Section choix du mode de prise de vue */}
      <div className="mode-title fade-in-up" style={{ marginTop: '30px' }}>
        <h2>Mode de prise de vue :</h2>
      </div>
      <div className="button-row fade-in-up">
        <button
          className={`base-button ${shootingMode === "auto" ? "selected-button" : ""}`}
          onClick={() => handleShootingSelection("auto")}
        >
          Automatique
        </button>

        <button
          className={`base-button ${shootingMode === "manual" ? "selected-button" : ""}`}
          onClick={() => handleShootingSelection("manual")}
        >
          Manuel
        </button>

        <button
          className={`base-button ${shootingMode === "file" ? "selected-button" : ""}`}
          onClick={() => handleShootingSelection("file")}
        >
          À partir d'un fichier
        </button>
      </div>

      {/* Bouton Exécuter */}
      <div className="button-row fade-in-up" style={{ marginTop: '40px' }}>
        <button
          className="button-row button"
          onClick={handleExecute}
          disabled={!canExecute()}
        >
          Exécuter
        </button>
      </div>
    </>
  );
}

export default App;