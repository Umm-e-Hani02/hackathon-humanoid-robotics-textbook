
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
  const [selectedText, setSelectedText] = useState('');
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

  // Detect text selection on the page
  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('keyup', handleTextSelection);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('keyup', handleTextSelection);
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
    const questionText = inputValue;
    const contextText = selectedText;
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      console.log('Sending request to:', apiUrl);
      console.log('Request payload:', { text: questionText, selected_text: contextText });

      const response = await fetch(apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: questionText,
          selected_text: contextText || null
        }),
      });

      console.log('Response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('Error response:', errorText);
        throw new Error(`Server error: ${response.status}. Please check the backend logs.`);
      }

      const data = await response.json();
      console.log('Response data:', data);

      const assistantMessage = { id: Date.now() + 1, text: data.answer, sender: 'assistant' };
      setMessages((prevMessages) => [...prevMessages, assistantMessage]);

      // Clear selected text after sending
      setSelectedText('');

    } catch (err) {
      console.error('Chat error:', err);
      setError(err.message);
      const errorMessage = {
        id: Date.now() + 1,
        text: `Error: ${err.message}. Check console for details.`,
        sender: 'assistant',
        isError: true
      };
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
            <h3>AI Assistant</h3>
            <button className="chatbot-close-btn" onClick={toggleChat} aria-label="Close chat">
              <svg
                xmlns="http://www.w3.org/2000/svg"
                width="18"
                height="18"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>
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

          {selectedText && (
            <div className="selected-text-indicator">
              <div className="selected-text-header">
                <span>📄 Selected text will be used as context</span>
                <button
                  className="clear-selection-btn"
                  onClick={() => setSelectedText('')}
                  aria-label="Clear selection"
                >
                  ✕
                </button>
              </div>
              <div className="selected-text-preview">
                {selectedText.length > 100
                  ? `${selectedText.substring(0, 100)}...`
                  : selectedText}
              </div>
            </div>
          )}

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
