// physical-ai-and-humanoid-robots/src/components/Chatbot/MessageDisplay.js
import React, { useEffect, useRef } from 'react';
import clsx from 'clsx'; // Ensure clsx is imported
import styles from './Chatbot.module.css'; // Import the CSS module

function MessageDisplay({ messages }) {
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  return (
    <div className={styles.messageDisplay}>
      {messages.map((msg, index) => (
        <div
          key={index}
          className={clsx(styles.message, {
            [styles.userMessage]: msg.sender === 'user',
            [styles.botMessage]: msg.sender !== 'user',
          })}
        >
          <span
            className={clsx(styles.messageBubble, {
              [styles.userMessageBubble]: msg.sender === 'user',
              [styles.botMessageBubble]: msg.sender !== 'user',
            })}
          >
            {msg.text}
          </span>
        </div>
      ))}
      <div ref={messagesEndRef} />
    </div>
  );
}

export default MessageDisplay;
