import React, { useState, useEffect, useCallback, useRef } from 'react';
import '../assets/CSS/Manual.css';
import droneConnectedIcon from '../assets/images/droneConnected.svg';
import logo from '../assets/images/logo.png';
import { io } from 'socket.io-client';




const socket = io('http://127.0.0.1:9080', {
    transports: ['websocket'], 
    autoConnect: true, // Automatically connects initially// 2-second delay between reconnections
});

const Manual = () => {

    const [isConnected, setIsConnected] = useState(false); // State for connection status
    const [isArmed, setIsArmed] = useState(false); // State to track if the drone is armed
    const [isGeotagging, setIsGeotagging] = useState(false);
    const [croppedImageUrl, setCroppedImageUrl] = useState('');
    const [imageUrls, setImageUrls] = useState([]);
    const [selectedImageUrl, setSelectedImageUrl] = useState('');
    const [buttons, setButtons] = useState([]);
    const [selectedMode, setSelectedMode] = useState(null);
    const [selectedImageIndex, setSelectedImageIndex] = useState(0); 
    const [backendData, setBackendData] = useState({
        voltage: '',
        current: '',
        temperature: '',
        latitude: '',
        longitude: ''
    });
    const [isLive, setIsLive] = useState(true);
    const [intervalId, setIntervalId] = useState(null);
    const [logs, setLogs] = useState([]); 
    const logContainerRef = useRef(null);// Add this line

    useEffect(() => {
        if (logContainerRef.current) {
            logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
        }
    }, [logs]);


    useEffect(() => {
        // Event handler for socket connection
        const handleConnect = () => console.log('Socket connected');

        // Event handler for socket disconnection
        const handleDisconnect = () => console.log('Socket disconnected');

        // Event handler for reconnect attempts
        const handleReconnectAttempt = () => console.log('Attempting to reconnect...');

        // Event handler for connection errors
        const handleConnectError = (error) => console.error('Connection error:', error);

        // Event handler for log messages from the server
        const handleLogMessage = (data) => {
            setLogs((prevLogs) => [...prevLogs, data.message]);
        };

        // Register event listeners
        socket.on('connect', handleConnect);
        socket.on('disconnect', handleDisconnect);
        socket.on('reconnect_attempt', handleReconnectAttempt);
        socket.on('connect_error', handleConnectError);
        socket.on('log_message', handleLogMessage);

        // Cleanup function to remove event listeners on unmount
        return () => {
            socket.off('connect', handleConnect);
            socket.off('disconnect', handleDisconnect);
            socket.off('reconnect_attempt', handleReconnectAttempt);
            socket.off('connect_error', handleConnectError);
            socket.off('log_message', handleLogMessage);
        };
    }, [socket]);

    const fetchImageUrls = useCallback((urls) => {
        const formattedUrls = urls.map(url => `http://192.168.144.50:9080/images/${url}`);

        console.log("raw urls from the backend: " , urls)
        console.log("Formatted urls :" ,formattedUrls)
        
        // huge console command
        console.log("Fecting images ..");
        fetch('http://192.168.144.50:9080/all-images')
        .then(res => {
            console.log(" status :", res.status)
            return res.text();
        })
        .then(text => {
            console.log("raw backend response:", text)
            try{
                const data = JSON.parse(text);
                console.log("Parse JSON:", data);
                fetchImageUrls(data);
            } catch (e){
                console.error("Failed", e);
            }
        })
        .catch(err => {
            console.error("Error fecting images :" ,err);
        });

        //*************** */
        // Set image URLs and automatically update to the latest image
        setImageUrls(formattedUrls);
        if (formattedUrls.length > 0) {
            setSelectedImageUrl(formattedUrls[formattedUrls.length - 1]);
           
        }
    }, []);

    useEffect(() => {
    // Define the polling interval
    const pollingInterval = 5000; // 5 seconds

    const fetchLatestImages = async () => {
        const response = await fetch('http://192.168.144.50:9080/all-images');
        const data = await response.json();
        fetchImageUrls(data.imageUrls);
    };

    // Fetch images at the start and then set an interval to poll the server
    fetchLatestImages();
    const intervalId = setInterval(fetchLatestImages, pollingInterval);

    return () => clearInterval(intervalId); // Clear interval on unmount
}, [fetchImageUrls]);

    useEffect(() => {
        const handleEscPress = (event) => {
            if (event.key === 'Escape') {
                closeModal();
            }
        };

        document.addEventListener('keydown', handleEscPress);
        
        return () => {
            document.removeEventListener('keydown', handleEscPress);
        };
    }, []);

    const handleKeyPress = useCallback((event) => {
        if (event.key === 'ArrowLeft') {
            // Navigate to the previous image
            setSelectedImageIndex((prevIndex) => Math.max(prevIndex - 1, 0));
        } else if (event.key === 'ArrowRight') {
            // Navigate to the next image
            setSelectedImageIndex((prevIndex) => Math.min(prevIndex + 1, imageUrls.length - 1));
        }
    }, [imageUrls]);

    useEffect(() => {
        document.addEventListener('keydown', handleKeyPress); // Add event listener

        return () => {
            document.removeEventListener('keydown', handleKeyPress); // Clean up
        };
    }, [handleKeyPress]);

    useEffect(() => {
        // Update selectedImageUrl when selectedImageIndex changes
        if (imageUrls.length > 0) {
            setSelectedImageUrl(imageUrls[selectedImageIndex]);
        }
    }, [selectedImageIndex, imageUrls]);


    // const handleImageClick = (url) => {
    //     setSelectedImageUrl(url);
    //     setIsLive(false);
    // };
    

    // const handleLiveButtonClick = () => {
    //     setIsLive(true);
    //     if (imageUrls.length > 0) {
    //         setSelectedImageUrl(imageUrls[imageUrls.length - 1]);
    //     }
    // };

    const openImage = (event) => {
        if (selectedImageUrl) {
            const modal = document.getElementById('image-modal');
            const modalImg = document.getElementById('modal-image');
            if (modal && modalImg) {
                modal.style.display = 'block';
                modalImg.src = selectedImageUrl;
                modalImg.style.maxWidth = '1100px';
                modalImg.style.maxHeight = '700px';
            }
        }
    };

    const closeModal = () => {
        const modal = document.getElementById('image-modal');
        if (modal) {
            modal.style.display = 'none';
        }
    };

    const cropImage = async (x, y) => {
        try {
            const response = await fetch('http://192.168.144.50:9080/crop-image', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ imageUrl: selectedImageUrl.split('/').pop(), x, y })
            });
            if (response.ok) {
                const data = await response.json();
                setCroppedImageUrl(data.croppedImageUrl);
            } else {
                console.error('Failed to crop image');
            }
        } catch (error) {
            console.error('Error cropping image:', error);
        }
    };

    const cropClick = (event) => {
        const modalImg = document.getElementById('modal-image');
        if (!modalImg) return;

        const rect = modalImg.getBoundingClientRect();
        const x = Math.floor((event.clientX - rect.left) / (rect.width / modalImg.naturalWidth));
        const y = Math.floor((event.clientY - rect.top) / (rect.height / modalImg.naturalHeight));

        cropImage(x, y);
    };

    const handleSaveButtonClick = async () => {
        try {
            const shape = document.querySelector('input[placeholder="Shape"]').value;
            // const colour = document.querySelector('input[placeholder="Colour"]').value;
    
            // Prepare button data but don't update the state yet
            const buttonData = {
                id: buttons.length + 1, // Create a unique ID for each button
                label: `${shape}`, // You can modify this to use any input data
            };
    
            // Make the POST request to save the details
            const response = await fetch('http://192.168.144.50:9080/save-details', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    selectedImageUrl,
                    croppedImageUrl,
                    shape,
                    // colour,
                    // coordinates: backendData.coordinates,
                    id: buttonData.id,  // Include button id
                    label: buttonData.label
                })
            });
    
            // Check if the response was successful
            if (!response.ok) {
                throw new Error('Failed to save details');
            }
    
            const result = await response.json(); // Parse JSON response
    
            // Proceed only if latitude and longitude exist in the result
            if (result.latitude && result.longitude) {
                // Update backendData with latitude and longitude
                setBackendData(prevData => ({
                    ...prevData,
                    latitude: result.latitude,
                    longitude: result.longitude
                }));
    
                // Update buttons state with the new button data
                setButtons(prevButtons => [
                    ...prevButtons,
                    { id: buttonData.id, label: buttonData.label } // Add button data only if latitude and longitude exist
                ]);
                console.log('Details saved successfully and backend data updated');
            } else {
                console.error('Latitude and longitude not returned from backend');
            }
            
        } catch (error) {
            console.error('Error saving details:', error);
        }
    };
    
    
// --Pymavlink-----------------------------------------------------

    const handleToggleGeotag = () => {
        setIsGeotagging(!isGeotagging);
    };


    // Function to toggle connection state
    const handleDroneConnection = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/toggle-connection', { method: 'POST' });
            if (response.ok) {
                console.log("toggle connection reporting manual.jsx")
                const data = await response.json();
                setIsConnected(true); // Assuming the connection is successful
    
            // Start continuous polling for drone status
            // const statusInterval = setInterval(async () => {
            //     try {
            //         const statusResponse = await fetch('http://192.168.144.50:9080/drone-status', { method: 'GET' });
            //         if (statusResponse.ok) {
            //             const statusData = await statusResponse.json();
            //             setSelectedMode(statusData.current_mode); // Update the selected mode
            //             setIsArmed(statusData.is_armed); // Update the armed status
            //         } else {
            //             console.error('Failed to get drone status');
            //         }
            //     } catch (error) {
            //         console.error('Error fetching drone status:', error);
            //     }
            // }, 1000); // Fetch status every 1 second (adjust as needed)
            // // Optionally store the interval ID so you can clear it later
            // return () => clearInterval(statusInterval);
            } else {
                console.error('Failed to toggle connection');
            }
        } catch (error) {
            console.error('Error toggling connection:', error);
        }
    };
    

    const handleArmClick = async () => {
        try {
            const action = isArmed ? 'disarm' : 'arm'; // Decide action based on current state
    
            const response = await fetch('http://192.168.144.50:9080/arm-disarm', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ action }), // Send action ('arm' or 'disarm') in the body
            });
    
            const data = await response.json();
            if (response.ok) {
                setIsArmed(!isArmed); // Toggle armed state if request succeeds
                // Optionally show success message
                // alert(data.message);
            } else {
                alert(`Error: ${data.message}`); // Show error message from backend
            }
        } catch (error) {
            console.error('Error in arm/disarm:', error);
            alert('An error occurred. Please try again.');
        }
    };

    const handleTakeoff = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/takeoff', { method: 'POST' });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error taking off:', error);
        }
    };

    const handleRTL = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/RTL', { method: 'POST' });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error taking off:', error);
        }
    };

    const handleDrop = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/drop', { method: 'POST' });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error dropping:', error);
        }
    };

    const autorepo = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/auto-repo', { method: 'POST', mode: 'no-cors' });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error stopping:', error);
        }
    };
    
    const dengrepo1 = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/dengu-repo', { method: 'POST', mode: 'no-cors' });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error stopping:', error);
        }
    };

    const dengrepo2 = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/dengu-repo-2', { method: 'POST', mode: 'no-cors' });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error stopping:', error);
        }
    };

    // const runAI = async () => {
    //     try {
    //         const response = await fetch('http://192.168.144.50:9080/runAI', { method: 'POST', mode: 'no-cors' });
    //         const data = await response.json();
    //         console.log(data.message);
    //     } catch (error) {
    //         console.error('Error stopping:', error);
    //     }
    // };

    const handleLock = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/lock-servo', { method: 'POST' });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error dropping:', error);
        }
    };

    const handleModeChange = async (mode) => {
        try {
            const response = await fetch('http://192.168.144.50:9080/change-mode', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ mode }),  // Send the selected mode to the backend
            });

            const data = await response.json();
            if (response.ok) {
                setSelectedMode(mode);  // Update the selected mode state
                // alert(data.message);  // Optional: Show the backend message
            } else {
                alert(`Error: ${data.message}`);  // Show error if the request fails
            }
        } catch (error) {
            console.error('Error changing mode:', error);
            alert('An error occurred. Please try again.');
        }
    };


    const startGeotag = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/start_geotagg', { method: 'POST', mode: 'no-cors' });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error starting off:', error);
        }
    };

    const startGeotagG = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/start_geotaggG', { method: 'POST', mode: 'no-cors' });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error starting off:', error);
        }
    };

    const stopGeotag = async () => {
        try {
            const response = await fetch('http://192.168.144.50:9080/stop_geotagg', { method: 'POST', mode: 'no-cors' });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error stopping:', error);
        }
    };


    const automationButton = async (id) => {
        try {
            const response = await fetch('http://192.168.144.50:9080/reposition', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ id }), // Include the id in the request body
            });
            const data = await response.json();
            console.log(data.message);
        } catch (error) {
            console.error('Error:', error);
        }
    };
    

// --Pymavlink-----------------------------------------------------

    return (
        <div className="manual-container">
            <div className="header-container">
                <header className="header">
                    <img src={logo} alt="Logo" className="logoM" />
                    <h1>Manual Flight</h1>
                    <button className={`connection-button ${isConnected ? 'green' : ''}`} onClick={handleDroneConnection}>
                        <img src={droneConnectedIcon} alt="Drone Connection" />
                        <span className="connect-text">{isConnected ? 'Connected' : 'Connect'}</span>
                    </button>
                </header>
            </div>
            <div className="content-container">
                <div className="left-container">
                    <div className="image-box">
                        {/* <div className="imageGrid">
                            {/* <div className="grid-container">
                                    {imageUrls.map((url, index) => (
                                        <img
                                            key={index}
                                            src={url}
                                            alt={`Grid Image ${index}`}
                                            onClick={() => handleImageClick(url)}
                                            className={selectedImageUrl === url ? 'selected' : ''}
                                        />
                                    ))}
                            </div> 
                            <div className="liveButtonContainer">
                                    <button 
                                        className="liveButton"
                                        onClick={handleLiveButtonClick}
                                    >
                                        Live
                                    </button>
                            </div>
                        </div> */}
                        <div className="mainImage">
                            {selectedImageUrl && (
                                    <img
                                        src={selectedImageUrl}
                                        alt="Main Image"
                                        onClick={openImage}
                                    />
                                )}
                        </div>
                        <div className="croppedImage">
                        <div className="croppedImageDisplay">
                                {croppedImageUrl && (
                                    <img src={croppedImageUrl} alt="Cropped Image" />
                                )}
                            </div>
                            <div className="inputsContainer">
                                <input className="inputBox" type="text" placeholder="Shape" />
                               
                                <input type="text" placeholder="Coordinates" className="coordinatesBox" value={`${backendData.latitude}, ${backendData.longitude}`} readOnly />
                                <button className="saveButton" onClick={handleSaveButtonClick}>Store</button>
                                <div className="repo-button-grid">
                                {buttons.map((button) => (
                                <button className='Repos-Button'
                                    key={button.id} 
                                    onClick={() => automationButton(button.id)} // Attach onClick function
                                >
                                    <span>{button.label}</span>
                                </button>
                                ))}
                                </div>
                            </div>
                        </div>
                    </div>
                    {/* <div className="terminal-box">
                    <div className="image_processing" id="image_processing">
                            {logs.map((log, index) => (
                                <div key={index}>{log}</div> // Render logs
                            ))}
                        </div>

                        <div className="drone_Status">
                            {/* Content for Drone Status*
                        </div>
                    </div> */}
                </div>
                <div className="right-container">
                    <div className="top-right-container">
                        <div className="button-grid">
                        {/* <button
                            className={`Control-Button ${isArmed ? 'armed' : 'disarmed'}`}  
                            onClick={handleArmClick}  >
                            {isArmed ? 'Armed' : 'Disarmed'}  
                        </button>
                            <button className="Control-Button" onClick={handleTakeoff}>Take Off</button>*/}
                            <button
                                onClick={startGeotag}
                            > Start Geotag </button>

                             <button
                                onClick={startGeotagG}
                            >
                                Geotag
                            </button>  

                            <button
                                onClick={stopGeotag}
                            >
                                Stop Geotag
                            </button> 
                            <button className="Control-Button" onClick={autorepo}>DROP ALL</button>
                            {/* <button className="Control-Button" onClick={autorepo}>Run AI</button> */}
                            <button className="Control-Button" onClick={handleDrop}>Drop</button>
			                <button className="Control-Button" onClick={dengrepo1}>DENG1</button>
                            <button className="Control-Button" onClick={dengrepo2}>DENG2</button>
                            <button className="Control-Button" onClick={handleRTL}>RTL</button>
                            <button
                                className={`Control-Button ${selectedMode === 'STABILIZE' ? 'active-mode' : ''}`}  // Apply 'active-mode' class if selected
                                onClick={() => handleModeChange('stabilize')}
                            >
                                Stabilize
                            </button>
                            <button
                                className={`Control-Button ${selectedMode === 'GUIDED' ? 'active-mode' : ''}`}  // Apply 'active-mode' class if selected
                                onClick={() => handleModeChange('guided')}
                            >
                                Guided
                            </button>
                            <button
                                className={`Control-Button ${selectedMode === 'AUTO' ? 'active-mode' : ''}`}  // Apply 'active-mode' class if selected
                                onClick={() => handleModeChange('auto')}
                            >
                                Auto
                            </button>
                            <button
                                className={`Control-Button ${selectedMode === 'LOITER' ? 'active-mode' : ''}`}  // Apply 'active-mode' class if selected
                                onClick={() => handleModeChange('loiter')}
                            >
                                Loiter
                            </button>
                            <button className="Control-Button" onClick={handleLock}>Lock Servo</button>
                        </div>
                    </div>
                    <div className="bottom-right-container">
                        <div className="bottom-up-container">
                        <div className="terminal-box" ref={logContainerRef}>
                            {logs.map((log, index) => (
                                <div key={index}>{log}</div> // Display each log message
                            ))}
                        </div>
                        
                        {/* <div className="terminal-box-2" ref={logContainerRef}>
                            {logs.map((log, index) => (
                                <div key={index}>{log}</div> // Display each log message
                            ))}
                        </div> */}
                            
                            {/* <div className="data-box">
                                <input type="text" placeholder="Voltage" value={backendData.voltage} readOnly />
                            </div>
                            <div className="data-box">
                                <input type="text" placeholder="Current" value={backendData.current} readOnly />
                            </div>
                            <div className="data-box">
                                <input type="text" placeholder="Temperature" value={backendData.temperature} readOnly />
                            </div> */}
                            
                        </div>
                        <div className="bottom-down-container">
                            
                        </div>
                    </div>
                </div>
            </div>
            <div id="image-modal" className="image-modal" onClick={closeModal}>
                <span className="close-modal" onClick={closeModal}>&times;</span>
                <img
                    id="modal-image"
                    src={selectedImageUrl}
                    alt="Enlarged Image"
                    className="modal-content"
                    onClick={(e) => {
                        e.stopPropagation();
                        cropClick(e);
                    }}
                />
            </div>
        </div>
    );
};

export default Manual;


