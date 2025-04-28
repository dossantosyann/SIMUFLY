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
  const [videoConnected, setVideoConnected] = useState(false);
  const videoRef = useRef(null);

  useEffect(() => {
    // Fonction pour vérifier si l'image est chargée correctement
    const checkVideoConnection = () => {
      const img = videoRef.current;
      if (img) {
        img.onerror = () => {
          setVideoConnected(false);
        };
        img.onload = () => {
          setVideoConnected(true);
        };
      }
    };
    
    checkVideoConnection();
    
    // Vérifier régulièrement la connexion vidéo
    const interval = setInterval(() => {
      // Forcer le rechargement de l'image pour vérifier à nouveau
      if (videoRef.current) {
        const timestamp = new Date().getTime();
        videoRef.current.src = `http://localhost:8080/video_feed?t=${timestamp}`;
      }
    }, 10000); // Vérifier toutes les 10 secondes
    
    return () => clearInterval(interval);
  }, []);

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

  return (
    <>
      <h1>Simulateur de prise de vue par drone</h1>

      {/* Flux vidéo avec message de fallback */}
      <div className="video-container">
        {videoConnected ? (
          <img 
            ref={videoRef}
            src="http://localhost:8080/video_feed" 
            alt="Flux vidéo en direct" 
            onError={() => setVideoConnected(false)}
          />
        ) : (
          <div className="video-placeholder">
            <p>Aucun flux vidéo disponible</p>
          </div>
        )}
      </div>

      {/* Le reste du code reste inchangé */}
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