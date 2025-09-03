import React from 'react';
import '/src/assets/CSS/Actions.css'; // Adjust the path based on your project structure

const Actions = () => {
    return (
        <div className="actions-container"> {/* Wrap your Actions component with a container */}
            <ul className="actions-list">
                <li><a href="\manual" target='_blank' data-text="Manual ">Manual</a></li>
                <li><a href="\ai" target='_blank' data-text="AI">AI</a></li>
                <li><a href="\camera" target='_blank' data-text="Camera">Camera</a></li>
            </ul>
        </div>
    );
};

export default Actions;
