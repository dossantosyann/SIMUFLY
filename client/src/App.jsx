import { useState, useEffect, useRef } from 'react'
import heigLogo from './assets/heigLogo.svg'
import './App.css'
import axios from 'axios';

function App() {
  const [movementMode, setMovementMode] = useState("");
  const [shootingMode, setShootingMode] = useState("");
  const [autoMovementParam, setAutoMovementParam] = useState("");
  const [selectedFile, setSelectedFile] = useState(null);
  const [fileName, setFileName] = useState("Aucun fichier sélectionné");
  const [stepX, setStepX] = useState("");
  const [stepY, setStepY] = useState("");
  const [videoError, setVideoError] = useState(false);
  const videoRef = useRef(null);

  // Ajout d'un gestionnaire d'erreur pour l'image/vidéo
  const handleVideoError = () => {
    setVideoError(true);
  };

  // Essayer à nouveau de charger la vidéo si elle devient disponible
  const retryVideo = () => {
    setVideoError(false);
    if (videoRef.current) {
      videoRef.current.src = `http://localhost:8080/video_feed?timestamp=${new Date().getTime()}`;
    }
  };

  // Le reste de votre code reste inchangé...

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
    console.log("Exécution avec mode de déplacement:", movementMode, "et mode de prise de vue:", shootingMode);

    const formData = new FormData();
    formData.append('movementMode', movementMode);
    formData.append('shootingMode', shootingMode);

    if (movementMode === "auto") {
      formData.append('autoMovementParam', autoMovementParam);
    } else if (movementMode === "file" && selectedFile) {
      formData.append('file', selectedFile);
    } else if (movementMode === "step") {
      formData.append('stepX', stepX);
      formData.append('stepY', stepY);
    }

    const config = {
      headers: {
        'Content-Type': 'multipart/form-data'
      }
    };

    axios.post('http://localhost:8080/execute', formData, config)
      .then(response => {
        console.log("Réponse du serveur:", response.data);
        // Réessayer de charger la vidéo après l'exécution
        retryVideo();
      })
      .catch(error => {
        console.error("Erreur lors de l'exécution:", error);
      });
  };

  const handleMove = () => {
    if (!stepX || !stepY) return;

    const moveData = {
      movementMode: "step",
      stepX: parseInt(stepX),
      stepY: parseInt(stepY),
      flag: "step_movement_flag"
    };

    axios.post('http://localhost:8080/move_step', moveData)
      .then(response => {
        console.log("Réponse du serveur:", response.data);
        // Réessayer de charger la vidéo après le déplacement
        retryVideo();
      })
      .catch(error => {
        console.error("Erreur lors du déplacement:", error);
      });
  };

  const handleCapture = () => {
    if (!stepX || !stepY) return;

    const moveData = {
      movementMode: "step",
      flag: "step_capture_flag"
    };

    axios.post('http://localhost:8080/capture_step', moveData)
      .then(response => {
        console.log("Réponse du serveur:", response.data);
      })
      .catch(error => {
        console.error("Erreur lors de la capture:", error);
      });
  };

  const canExecute = () => {
    if (movementMode === "" || shootingMode === "") return false;
    if (movementMode === "auto" && autoMovementParam === "") return false;
    if (movementMode === "file" && !selectedFile) return false;
    return true;
  };

  const handleGenerateFile = async () => {
    try {
      const response = await axios.post('http://localhost:8080/generate_file', {}, {
        responseType: 'blob', // Important pour bien récupérer le fichier binaire
      });

      const blob = new Blob([response.data], { type: 'application/json' });
      const url = window.URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.setAttribute('download', 'deplacement.json');
      document.body.appendChild(link);
      link.click();
      link.parentNode.removeChild(link);
    } catch (error) {
      console.error('Erreur lors de la génération du fichier:', error);
    }
  };


  return (
    <>
      <h1>Simulateur de prise de vue par drone</h1>

      {/* Flux vidéo avec message de fallback */}
      <div className="video-container">
        {videoError ? (
          <div className="video-placeholder">
            <div className="placeholder-text">
              Aucun flux vidéo disponible
            </div>
          </div>
        ) : (
          <img
            ref={videoRef}
            src="http://localhost:8080/video_feed"
            alt="Flux vidéo en direct"
            onError={handleVideoError}
          />
        )}
      </div>

      {/* Le reste du code reste inchangé */}
      <div className="mode-title">
        <h2>Mode de déplacement :</h2>
      </div>

      {/* Votre code existant continue ici... */}
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

      {/* Le reste de votre code JSX ici... */}
      {movementMode === "auto" && (
        <div className="parameter-zone fade-in-up">
          <label htmlFor="autoParam">Nombre d'images à capturer :</label>
          <input
            type="text"
            id="autoParam"
            value={autoMovementParam}
            onChange={(e) => {
              const val = e.target.value;
              const filteredVal = val.replace(/[^0-9]/g, '');
              if (filteredVal === "" || parseInt(filteredVal) > 0) {
                setAutoMovementParam(filteredVal);
              }
            }}
            placeholder="Entrez un nombre"
          />
          <button
            className='generate-button'
            style={{ marginTop: '10px' }}
            onClick={handleGenerateFile}
          >
            Générer le fichier de déplacement
          </button>
        </div>
      )}

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
                if (val.includes('-')) {
                  val = '-' + val.replace(/-/g, '');
                }
                setStepX(val);
              }}
              placeholder="X"
            />
            <input
              type="text"
              value={stepY}
              onChange={(e) => {
                let val = e.target.value.replace(/[^0-9-]/g, '');
                if (val.includes('-')) {
                  val = '-' + val.replace(/-/g, '');
                }
                setStepY(val);
              }}
              placeholder="Y"
            />
            <button
              onClick={handleMove}
              disabled={!stepX || !stepY}
              className="move-button"
              style={{
                marginLeft: "10px",
                padding: "8px 16px",
                backgroundColor: !stepX || !stepY ? "#cccccc" : "#4CAF50"
              }}
            >
              Déplacer
            </button>
            <button
              onClick={handleCapture}
              disabled={!stepX || !stepY}
              className="capture-button"
              style={{
                marginLeft: "10px",
                padding: "8px 16px",
                backgroundColor: !stepX || !stepY ? "#cccccc" : "#4CAF50"
              }}
            >
              Capturer
            </button>
          </div>
        </div>
      )}

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