import React from 'react';
import logo from '/src/assets/images/logo.png';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faCog } from '@fortawesome/free-solid-svg-icons';

const Header = () => {
    return (
        <header className="header">
            <div className="logo-container">
                <img src={logo} alt="Logo" className="logo" />
            </div>
            <div className="settings-container">
                <button className="settings-button">
                    <FontAwesomeIcon icon={faCog} />
                </button>
            </div>
        </header>
    );
};


export default Header;
