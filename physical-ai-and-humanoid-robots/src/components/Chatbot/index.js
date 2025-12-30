import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

export default function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: 'Hi! How can I help you today?', sender: 'bot', timestamp: new Date() }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setIsTyping(true);

    try {
      // Simulate API call to chatbot backend
      // In a real implementation, this would call the actual API
      setTimeout(() => {
        const responses = [
          'That\'s an interesting question about Physical AI! Let me provide you with some insights from the textbook.',
          'Based on the Physical AI and Humanoid Robotics concepts, here\'s what I can tell you...',
          'Great question! Physical AI combines robotics, machine learning, and physics to create more capable robots.',
          'The textbook covers this topic in detail. Here\'s a summary of the key concepts...',
          'According to the principles of Physical AI, the answer involves understanding both hardware and software integration.'
        ];
        const randomResponse = responses[Math.floor(Math.random() * responses.length)];

        const botResponse = {
          id: Date.now() + 1,
          text: randomResponse,
          sender: 'bot',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, botResponse]);
        setIsLoading(false);
        setIsTyping(false);
      }, 1500);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot',
        timestamp: new Date(),
        isError: true
      };
      setMessages(prev => [...prev, errorMessage]);
      setIsLoading(false);
      setIsTyping(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  // Format timestamp to show time
  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  // Quick reply options
  const quickReplies = [
    "What is Physical AI?",
    "Humanoid Robotics basics",
    "AI in robotics",
    "Learning resources"
  ];

  const handleQuickReply = (reply) => {
    setInputValue(reply);
  };

  return (
    <>
      {/* Floating Chat Icon */}
      {!isOpen && (
        <button
          className={styles.chatIcon}
          onClick={toggleChat}
          aria-label="Open chatbot"
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
            className={styles.chatIconSvg}
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}

      {/* Chatbot Window */}
      {isOpen && (
        <div className={styles.chatContainer}>
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <div className={styles.botAvatar}>
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  width="20"
                  height="20"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                >
                  <path d="M12 8V4H8"></path>
                  <rect width="16" height="12" x="4" y="8" rx="2"></rect>
                  <path d="M2 14h2"></path>
                  <path d="M20 14h2"></path>
                  <path d="M15 13v2"></path>
                  <path d="M9 13v2"></path>
                </svg>
              </div>
              <div className={styles.headerText}>
                <h3>Physical AI Assistant</h3>
                <p className={styles.statusText}>
                  <span className={styles.statusIndicator}></span>
                  Online
                </p>
              </div>
            </div>
            <button
              className={styles.closeButton}
              onClick={toggleChat}
              aria-label="Close chat"
            >
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

          <div className={styles.chatMessages}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${styles[message.sender]} ${message.isError ? styles.error : ''}`}
              >
                <div className={styles.messageContent}>
                  {message.text}
                </div>
                <div className={styles.messageTime}>
                  {formatTime(message.timestamp)}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.bot} ${styles.typingMessage}`}>
                <div className={styles.typingIndicator}>
                  <div className={styles.typingDot}></div>
                  <div className={styles.typingDot}></div>
                  <div className={styles.typingDot}></div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Quick Reply Buttons - only show if the initial message exists and no user has sent a message yet */}
          {messages.length === 1 && messages[0].sender === 'bot' && !isLoading && (
            <div className={styles.quickReplies}>
              {quickReplies.map((reply, index) => (
                <button
                  key={index}
                  className={styles.quickReplyButton}
                  onClick={() => handleQuickReply(reply)}
                >
                  {reply}
                </button>
              ))}
            </div>
          )}

          <form onSubmit={handleSendMessage} className={styles.chatInputForm}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Type a message..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={!inputValue.trim() || isLoading}
              aria-label="Send message"
            >
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
      )}
    </>
  );
}