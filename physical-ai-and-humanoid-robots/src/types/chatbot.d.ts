// physical-ai-and-humanoid-robots/src/types/chatbot.d.ts

export interface UserQuery {
  id: string;
  text: string;
  timestamp: string;
  sender: 'user';
}

export interface ChatbotResponse {
  id: string;
  query_id: string;
  response: string;
  timestamp: string;
  is_grounded: boolean;
  sender: 'bot';
  error_message?: string; // Optional, as it's only present for error messages
}

export interface Conversation {
  id: string;
  messages: Array<UserQuery | ChatbotResponse>;
  start_time: string;
  last_updated: string;
}
