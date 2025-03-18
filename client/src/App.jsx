import { useState, useEffect } from 'react'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
import './App.css'
import axios from 'axios';

function App() {
  const [count, setCount] = useState(0)
  const [array, setArray] = useState([])
  const [sliderValue1, setSliderValue1] = useState(50); // Valeur initiale du slider
  const [sliderValue2, setSliderValue2] = useState(50); // Valeur initiale du slider

  const fetchAPI = async () => {
    const response = await axios.get("http://localhost:8080/api/users");
    console.log(response.data.users);
    setArray(response.data.users);
  };

  useEffect(() => {
    fetchAPI();
  }, []);

  return (
    <>
      <div>
        <a href="https://vite.dev" target="_blank">
          <img src={viteLogo} className="logo" alt="Vite logo" />
        </a>
        <a href="https://react.dev" target="_blank">
          <img src={reactLogo} className="logo react" alt="React logo" />
        </a>
      </div>
      <h1>Vite + React</h1>
      <div className="card">
        <button onClick={() => setCount((count) => count + 1)}>
          count is {count}
        </button>
          {array.map((user, index) => (
            <div key={index}>
              <span>{user}</span>
              <br></br>
            </div>
          ))}
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

      <p className="read-the-docs">
        Click on the Vite and React logos to learn more
      </p>
    </>
  )
}

export default App
