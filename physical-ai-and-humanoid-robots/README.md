# Website

This website is built using [Docusaurus](https://docusaurus.io/), a modern
static website generator.

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window.
Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be
served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```


## RAG Agent Backend

This project includes a FastAPI backend for the RAG agent.

### Prerequisites

- An active backend virtual environment (e.g., activated via `backend/.venv/Scripts/Activate.ps1`).
- A running Qdrant instance accessible via the URL and API key configured in your `.env` file (`QDRANT_URL`, `QDRANT_API_KEY`).
- The Qdrant collection specified by `QDRANT_COLLECTION_NAME` (default: `physical-ai-book-rag`) must exist and be populated with data.

### How to Run

1.  **Activate your backend virtual environment**:
    ```bash
    backend\.venv\Scripts\Activate.ps1
    ```
2.  **Ensure Qdrant is running and populated**.
3.  **Run the FastAPI application**:
    ```bash
    uvicorn backend.main:app --reload
    ```

The application will be available at `http://127.0.0.1:8000`.

You can interact with the RAG agent by sending a POST request to the `/agent/chat` endpoint with a JSON payload like:
```json
{
  "text": "What are the core principles of Physical AI?"
}
```

## Retrieval Validation Tests

This project includes a suite of integration tests to validate the retrieval quality of the RAG pipeline against the Qdrant vector store. These tests perform similarity searches using sample queries and assert on the structure and metadata of the retrieved results.

### Prerequisites

- An active backend virtual environment (e.g., activated via `backend/.venv/Scripts/Activate.ps1`).
- A running Qdrant instance accessible via the URL and API key configured in your `.env` file (`QDRANT_URL`, `QDRANT_API_KEY`).
- The Qdrant collection specified by `QDRANT_COLLECTION_NAME` (default: `physical-ai-book-rag`) must exist and be populated with data.

### How to Run

1.  **Activate your backend virtual environment**:
    ```bash
    backend\.venv\Scripts\Activate.ps1
    ```
2.  **Ensure Qdrant is running and populated**.
3.  **Execute the tests using Pytest**:
    ```bash
    pytest tests/integration/test_retrieval_validation.py
    ```

The test output will include detailed logs for each query, showing the retrieved chunk IDs, relevance scores, and metadata.
