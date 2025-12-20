import React, { useState } from 'react';
import styles from './ChatbotWidget.module.css';
import ChatWindow from '../Chatbot/ChatWindow'; // Import ChatWindow

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      <div className={styles.chatbotIcon} onClick={toggleChat}>
        💬
      </div>
      {isOpen && (
        <div className={styles.chatbotWindow}>
          <ChatWindow /> {/* Render the ChatWindow component */}
        </div>
      )}
    </>
  );
};

export default ChatbotWidget;
