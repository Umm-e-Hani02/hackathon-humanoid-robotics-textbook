// physical-ai-and-humanoid-robots/src/components/Chatbot/ChatWindow.js
import React, { useState, useEffect, useRef } from 'react';
import { v4 as uuidv4 } from 'uuid';
import MessageInput from './MessageInput';
import MessageDisplay from './MessageDisplay';
import { sendMessage } from '../../services/chatbotApi';
import { UserQuery, ChatbotResponse, Conversation } from '../../types/chatbot';
import styles from './Chatbot.module.css'; // Import the CSS module

function ChatWindow() {
  const [conversation, setConversation] = useState(() => {
    const savedConversation = localStorage.getItem('chatbotConversation');
    return savedConversation ? JSON.parse(savedConversation) : {
      id: uuidv4(),
      messages: [],
      start_time: new Date().toISOString(),
      last_updated: new Date().toISOString(),
    };
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const conversationRef = useRef(conversation); // Ref to hold current conversation state for effects

  useEffect(() => {
    conversationRef.current = conversation; // Update ref whenever conversation changes
    localStorage.setItem('chatbotConversation', JSON.stringify(conversation));
  }, [conversation]);

  const handleSendMessage = async (text) => {
    setError(null);
    setIsLoading(true);

    const userMessage = {
      id: uuidv4(),
      text: text,
      timestamp: new Date().toISOString(),
      sender: 'user', // Add sender for MessageDisplay
    };

    setConversation(prevConv => {
      const newMessages = [...prevConv.messages, userMessage];
      return { ...prevConv, messages: newMessages, last_updated: new Date().toISOString() };
    });

    try {
      const responseData = await sendMessage(text, conversation.id);

      const botMessage = {
        id: uuidv4(),
        query_id: userMessage.id,
        response: responseData.response,
        timestamp: new Date().toISOString(),
        is_grounded: responseData.is_grounded,
        sender: 'bot', // Add sender for MessageDisplay
      };

      setConversation(prevConv => {
        const newMessages = [...prevConv.messages, botMessage];
        return { ...prevConv, messages: newMessages, last_updated: new Date().toISOString() };
      });

    } catch (err) {
      console.error('Failed to send message:', err);
      let errorMessage = 'An unexpected error occurred.';
      if (err.response && err.response.data && err.response.data.detail) {
        errorMessage = err.response.data.detail;
      } else if (err.message) {
        errorMessage = err.message;
      }
      setError(errorMessage);

      const errorMessageObj = {
        id: uuidv4(),
        query_id: userMessage.id,
        response: errorMessage,
        timestamp: new Date().toISOString(),
        is_grounded: false,
        sender: 'bot',
        error_message: errorMessage,
      };

      setConversation(prevConv => {
        const newMessages = [...prevConv.messages, errorMessageObj];
        return { ...prevConv, messages: newMessages, last_updated: new Date().toISOString() };
      });
    } finally {
      setIsLoading(false);
    }
  };

  // Filter messages for MessageDisplay component, converting UserQuery/ChatbotResponse to simple {sender, text}
  const displayMessages = conversation.messages.map(msg => ({
    sender: msg.sender,
    text: msg.text || msg.response || msg.error_message, // Use text for user, response or error_message for bot
  }));


  return (
    <div className={styles.chatbotWindowContent}>
      <MessageDisplay messages={displayMessages} />
      {isLoading && <div className={styles.chatbotLoadingIndicator}>Typing...</div>}
      {error && <div className={styles.chatbotErrorIndicator}>Error: {error}</div>}
      <MessageInput onSendMessage={handleSendMessage} />
    </div>
  );
}

export default ChatWindow;
