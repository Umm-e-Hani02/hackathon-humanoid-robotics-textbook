# Quickstart: Frontend Integration for RAG Chatbot

This quickstart guide provides a high-level overview of integrating the RAG Chatbot frontend into the Docusaurus project.

## 1. Backend Preparation

Ensure the existing FastAPI RAG backend is deployed and accessible. The backend must have CORS configured to allow requests from the Docusaurus frontend's domain (e.g., `https://physical-ai-and-humanoid-robots.github.io`).

## 2. Frontend Development (Docusaurus)

1.  **Create Chatbot Components**: Develop the necessary React components for the chatbot UI (e.g., `ChatbotWidget.js`, `ChatWindow.js`, `MessageInput.js`, `MessageDisplay.js`). These components should reside within the Docusaurus project's `physical-ai-and-humanoid-robots/src/components/Chatbot/` directory.

2.  **API Service Integration**: Create a service module (e.g., `physical-ai-and-humanoid-robots/src/services/chatbotApi.js`) to handle API calls to the RAG backend. This module will encapsulate the `fetch` or `axios` logic for sending user queries and receiving responses based on the defined `chat_api.json` contract.

3.  **Embed Chatbot Widget**: Integrate the `ChatbotWidget` component into the Docusaurus layout. This can be done by modifying a global layout component or injecting it into specific pages, ensuring it's visible on all book content pages as per `FR-001`. A common approach is to use Docusaurus's swizzling feature or custom themes.

4.  **State Management**: Implement client-side state management (e.g., using React's `useState`/`useReducer` or a context API) to manage the conversation history, user input, and loading states within the chatbot UI.

## 3. End-to-End Testing

1.  **Local Testing**: Run the Docusaurus project locally and ensure the chatbot UI is rendered correctly. Verify that sending queries from the chatbot successfully calls the local or deployed backend and displays responses.
2.  **Deployed Testing**: After deploying the Docusaurus site (e.g., to GitHub Pages), verify that the chatbot functions correctly on the live site, making sure CORS issues are resolved and the end-to-end flow from user query to chatbot response works as expected.

## 4. Key Files

-   `physical-ai-and-humanoid-robots/src/components/Chatbot/`: Directory for React components.
-   `physical-ai-and-humanoid-robots/src/services/chatbotApi.js`: API integration logic.
-   `backend/main.py`: (Potentially requires CORS configuration changes).
-   `specs/001-rag-chatbot-frontend/data-model.md`: Defines frontend data structures.
-   `specs/001-rag-chatbot-frontend/contracts/chat_api.json`: Defines the API contract.