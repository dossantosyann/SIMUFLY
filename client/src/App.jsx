import { useState, useEffect } from 'react'
import heigLogo from './assets/heigLogo.svg'
import './App.css'
import axios from 'axios';

function App() {
  const [count, setCount] = useState(0)
  const [array, setArray] = useState([])
  const [sliderValue1, setSliderValue1] = useState(50); // Valeur initiale du slider
  const [sliderValue2, setSliderValue2] = useState(50); // Valeur initiale du slider
  const [pictureFlag, setPictureFlag] = useState("");

  const fetchAPI = async () => {
    const response = await axios.get("http://localhost:8080/api/users");
    console.log(response.data.users);
    setArray(response.data.users);
  };

  const sendFlag = async () => {
    try { const response = await axios.post("http://localhost:8080/take_picture", {
        flag: "mon super flag",
      });
      setPictureFlag(response.data.pictureFlag);
    } catch(error) {
      console.error("Erreur :", error);
      setPictureFlag("Erreur lors de l'envoi du flag");
    }
  };

  useEffect(() => {
    fetchAPI();
  }, []);

  return (
    <>
      <div>
        <a href="https://heig-vd.ch" target="_blank">
          <img src={heigLogo} className="logo" alt="HEIG logo" />
        </a>
      </div>

      <div>
            <button onClick={sendFlag}>Envoyer le Flag</button>
            {pictureFlag && <p>{pictureFlag}</p>}
      </div>

      <h1>Drone simulator</h1>
      <div className="card">
        <button onClick={() => setCount((count) => count + 1)}>
          count is {count}
        </button>
      </div>

      <div className="video-container">
        <h2>Flux vidéo</h2>
        <img src="http://localhost:8080/video_feed" alt="Flux vidéo en direct" />
      </div>

      {/* Slider */}
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
    </>
  )
}

export default App
