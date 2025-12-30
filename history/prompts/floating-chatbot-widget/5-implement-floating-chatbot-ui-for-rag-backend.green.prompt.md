---
id: 5
title: Implement Floating Chatbot UI for RAG Backend
stage: green
date: 2025-12-22
surface: agent
model: gemini
feature: floating-chatbot-widget
branch: 001-rag-chatbot-frontend
user: admin
command: Build a fresh, minimal chatbot UI for the existing RAG backend.
labels: [chatbot, react, docusaurus, frontend]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - physical-ai-and-humanoid-robots/src/components/Chatbot/Chatbot.js
  - physical-ai-and-humanoid-robots/src/components/Chatbot/Chatbot.css
  - physical-ai-and-humanoid-robots/src/theme/Root.tsx
tests:
  - npm run start in physical-ai-and-humanoid-robots
  - npm run build in physical-ai-and-humanoid-robots
---

## Prompt

Build a fresh, minimal chatbot UI for the existing RAG backend.

Requirements:

- Frontend only (React + Docusaurus compatible)
- Do NOT modify backend or API logic
- Do NOT add a navbar item or separate page
- Do NOT use a header or title

UI Behavior:
- A small floating chat icon fixed at the bottom-right corner
- Clicking the icon opens a compact chat panel
- Clicking outside or on close icon hides the panel
- Chat must open and close reliably

UI Design:
- Clean, minimal, modern
- Small width, not full-screen
- Message bubbles (user / assistant)
- Input box + send button
- Loading indicator while waiting for response

Integration:
- Call the existing REST API
- Use fetch or axios
- Handle errors gracefully (show a simple message)

Constraints:
- Keep CSS minimal and scoped
- Must not break Docusaurus build
- Must work locally with `npm run start`

After completion, confirm:
- Chat icon is visible on all pages
- Chat opens/closes correctly
- Messages are sent and responses received
- Build and dev server run without errors

## Response snapshot

I have successfully built and integrated the floating chatbot UI into your Docusaurus project. The development server is running, and the build was successful. The chatbot is available on all pages and connects to your backend API.

## Outcome

- ✅ Impact: A new floating chatbot UI has been successfully added to the Docusaurus project, providing a user-friendly interface for the existing RAG backend.
- 🧪 Tests: The implementation was verified by running the Docusaurus development server (`npm run start`) and a production build (`npm run build`), both of which completed successfully.
- 📁 Files: Created `Chatbot.js` and `Chatbot.css` for the component, and modified `Root.tsx` to integrate it into the application.
- 🔁 Next prompts: The task is complete.
- 🧠 Reflection: The main challenge was resolving the module import path in the Docusaurus environment. After trying relative paths and different aliases, `@site/src/...` was found to be the correct alias for the project. This highlights the importance of understanding the specific project's build configuration.
