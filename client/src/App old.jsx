import { useState } from 'react'
import heigLogo from './assets/heigLogo.svg'
import './App.css'
import axios from 'axios';

function App() {
  const [movementMode, setMovementMode] = useState("");
  const [shootingMode, setShootingMode] = useState("");

  const handleMovementSelection = (mode) => {
    setMovementMode(mode);
  };

  const handleShootingSelection = (mode) => {
    setShootingMode(mode);
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
    </>
  );
}

export default App;






/*
Bouton flag
<div>
      <button onClick={sendFlag}>Envoyer le Flag</button>
      {pictureFlag && <p>{pictureFlag}</p>}
</div>
*/

/*
Compteur
<div className="card">
  <button onClick={() => setCount((count) => count + 1)}>
    count is {count}
  </button>
</div>
*/

/*
{ Slider }
<div className='sliders-container'>
<div className="slider-wrapper">
  <input
    type="range"
    min="0"
    max="100"
    value={sliderValue1}
    onChange={(e) => setSliderValue1(e.target.value)}
    className = "slider"
    />
  <p>Valeur du slider : {sliderValue1}</p>
</div>
<div className="slider-wrapper">
  <input
    type="range"
    min="0"
    max="100"
    value={sliderValue2}
    onChange={(e) => setSliderValue2(e.target.value)}
    className = "slider"
    />
  <p>Valeur du slider : {sliderValue2}</p>
</div>
</div>
*/