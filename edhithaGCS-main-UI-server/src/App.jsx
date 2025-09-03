
import './App.css'
import React from 'react';
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import CameraUi from './Pages/CameraUI'
import Home from './Pages/Home';
import AI from './Pages/AI';
import ManualPage from './Pages/Manual';


function App() {
  return (
    <div className='App'>
      <BrowserRouter>
        <Routes>
          <Route index element={<Home />} />
          <Route path='/manual' element={<ManualPage />} />
          <Route path='/ai' element={<AI />} />
          <Route path='/camera' element={<CameraUi />} />
        </Routes>
      </BrowserRouter>
    </div>
  );
}

export default App
