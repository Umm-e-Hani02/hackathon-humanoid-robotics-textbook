// physical-ai-and-humanoid-robots/src/components/Chatbot/MessageInput.js
import React, { useState } from 'react';
import styles from './Chatbot.module.css'; // Import the CSS module

function MessageInput({ onSendMessage }) {
  const [message, setMessage] = useState('');

  const handleSubmit = (event) => {
    event.preventDefault();
    if (message.trim()) {
      onSendMessage(message);
      setMessage('');
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.inputForm}>
      <input
        type="text"
        value={message}
        onChange={(e) => setMessage(e.target.value)}
        placeholder="Type your message..." // Updated placeholder
        className={styles.inputField}
      />
      <button
        type="submit"
        className={styles.sendButton}
        disabled={!message.trim()} // Disable when input is empty
      >
        Send
      </button>
    </form>
  );
}

export default MessageInput;
