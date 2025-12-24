
import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './Chatbot.css';

const Chatbot = () => {
  const { siteConfig } = useDocusaurusContext();
  const { apiUrl } = siteConfig.customFields;
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
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

    const userMessage = { text: inputValue, sender: 'user' };
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
      const assistantMessage = { text: data.answer, sender: 'assistant' };
      setMessages((prevMessages) => [...prevMessages, assistantMessage]);

    } catch (err) {
      setError(err.message);
      const errorMessage = { text: err.message, sender: 'assistant', isError: true };
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
          <div className="chatbot-header">
            <h3>Chat with our AI</h3>
            <button className="chatbot-close" onClick={toggleChat}>&times;</button>
          </div>
          <div className="chatbot-messages">
            {messages.map((msg, index) => (
              <div key={index} className={`message ${msg.sender} ${msg.isError ? 'error' : ''}`}>
                <p>{msg.text}</p>
              </div>
            ))}
            {isLoading && (
              <div className="message assistant">
                <div className="loading-indicator">
                  <span></span><span></span><span></span>
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
                placeholder="Type your message..."
                disabled={isLoading}
              />
              <button type="submit" disabled={isLoading}>Send</button>
            </form>
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;
