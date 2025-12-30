
import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './Chatbot.css';

const Chatbot = () => {
  const { siteConfig } = useDocusaurusContext();
  const { apiUrl } = siteConfig.customFields;
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: 'Hi, How can I help you today?', sender: 'assistant' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const chatbotPanelRef = useRef(null);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    const handleClickOutside = (event) => {
      if (chatbotPanelRef.current && !chatbotPanelRef.current.contains(event.target)) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const toggleChat = (e) => {
    e.stopPropagation();
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // NOTE: Replace with your actual API endpoint and payload structure
      const response = await fetch(apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ text: inputValue }),
      });

      if (!response.ok) {
        throw new Error('Something went wrong. Please try again.');
      }

      const data = await response.json();
      const assistantMessage = { id: Date.now() + 1, text: data.answer, sender: 'assistant' };
      setMessages((prevMessages) => [...prevMessages, assistantMessage]);

    } catch (err) {
      setError(err.message);
      const errorMessage = { id: Date.now() + 1, text: err.message, sender: 'assistant', isError: true };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };


  return (
    <div className="chatbot-container" ref={chatbotPanelRef}>
      {!isOpen && (
        <button className="chatbot-icon" onClick={toggleChat}>
          <span>🤖</span>
        </button>
      )}

      {isOpen && (
        <div className="chatbot-panel">
          <div className="chatbot-messages">
            {messages.map((msg, index) => (
              <div key={msg.id || index} className={`message ${msg.sender} ${msg.isError ? 'error' : ''}`}>
                <div className="message-content">
                  <p>{msg.text}</p>
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="message assistant typing-message">
                <div className="typing-indicator">
                  <div className="typing-dot"></div>
                  <div className="typing-dot"></div>
                  <div className="typing-dot"></div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input-area">
            <form onSubmit={handleSendMessage}>
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder="Type a message..."
                disabled={isLoading}
              />
              <button type="submit" disabled={isLoading || !inputValue.trim()}>
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  width="16"
                  height="16"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                >
                  <line x1="22" y1="2" x2="11" y2="13"></line>
                  <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                </svg>
              </button>
            </form>
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;
