// CameraUI.jsx
import React, { useState, useEffect } from 'react';
import '/src/assets/CSS/Cameraui.css';

const CameraUI = () => {
    const [thrust, setThrust] = useState(0);
    const [pwm, setPwm] = useState(0);
  
    useEffect(() => {
      const fetchData = async () => {
        try {
          const response = await fetch('http://127.0.0.1:9080/save-details');
          const data = await response.json();
          setThrust(data.latitude);
          setPwm(data.longitude);
        } catch (error) {
          console.error('Error fetching data:', error);
        }
      };
  
      const interval = setInterval(fetchData, 1000); // Fetch data every second
  
      return () => clearInterval(interval); // Cleanup interval on component unmount
    }, []);
  
    return (
      <div className="App">
        <h1>Thrust and PWM Monitor</h1>
        <div className="data-container">
          <div className="data-box">
            <h2>Thrust</h2>
            <p>{thrust}</p>
          </div>
          <div className="data-box">
            <h2>PWM</h2>
            <p>{pwm}</p>
          </div>
        </div>
      </div>
    );
};

export default CameraUI;
