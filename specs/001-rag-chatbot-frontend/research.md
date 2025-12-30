# Research: Frontend Integration for RAG Chatbot

## CORS Configuration in FastAPI

**Decision**: Implement CORS middleware in FastAPI to allow requests from the Docusaurus frontend domain.

**Rationale**: The Docusaurus frontend will be hosted on a different origin (e.g., GitHub Pages) than the FastAPI backend. To allow the browser to make API requests, the backend must explicitly permit cross-origin resource sharing.

**Alternatives Considered**:
-   **Proxying API requests through Docusaurus**: While possible, this adds unnecessary complexity to the Docusaurus build process and deployment for a simple API call.
-   **Disabling CORS entirely (not recommended for production)**: This would expose the API to all origins, which is a security risk.

## Embedding Custom React Components in Docusaurus

**Decision**: Develop the chatbot UI as a standalone React component within the Docusaurus project's `src/components` directory and integrate it into Docusaurus pages or layouts.

**Rationale**: Docusaurus is built on React, allowing seamless integration of custom React components. This approach leverages the existing Docusaurus infrastructure and React's component-based architecture for modular development.

**Alternatives Considered**:
-   **Using an iframe**: This would isolate the chatbot from the Docusaurus context, making communication more complex and potentially hindering styling consistency.
-   **Vanilla JavaScript widget**: Possible but would bypass React's benefits and likely result in a less maintainable solution within a React-based project.

## Handling API Calls from Docusaurus

**Decision**: Use the browser's native `fetch` API or a lightweight library like `axios` within the React components for making API calls to the FastAPI backend.

**Rationale**: `fetch` is a modern, promise-based API for making network requests and is readily available in all modern browsers. `axios` offers a slightly more feature-rich experience (e.g., automatic JSON parsing, request/response interceptors) but adds a small dependency. Both are suitable for a lightweight frontend.

**Alternatives Considered**:
-   **Server-Side Rendering (SSR) API calls**: Not applicable for a static Docusaurus site unless a custom server layer is introduced, which contradicts the "lightweight and embeddable" constraint.

## Chatbot UI Design Considerations for Docusaurus

**Decision**: Design a minimalist, unobtrusive chatbot UI that can be toggled open/closed, typically positioned as a floating widget (e.g., bottom-right corner) to avoid obstructing main content. Ensure responsive design for various screen sizes.

**Rationale**: To provide a seamless user experience, the chatbot should not interfere with reading the book content. A floating, collapsible widget is a common pattern for chatbots and integrates well with existing website layouts. Responsive design is crucial for accessibility across devices.

**Alternatives Considered**:
-   **Full-page chatbot interface**: This would be too disruptive to the Docusaurus reading experience.
-   **Chatbot integrated directly into content sections**: While possible for specific use cases, a general-purpose RAG chatbot should be universally accessible across all book content.