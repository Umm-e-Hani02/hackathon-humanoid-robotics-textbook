# Quickstart Guide: RAG Ingestion Pipeline

This guide provides basic steps to set up and run the RAG ingestion pipeline.

## 1. Prerequisites

Before you begin, ensure you have the following:

*   **Python 3.10+** installed.
*   **Access to a Docusaurus Website URL**: The live URL of the book you intend to ingest.
*   **Cohere API Key**: For generating text embeddings. You can obtain one from the Cohere website.
*   **Qdrant Cloud Account and API Key**: For storing vector embeddings. You can set up a cluster on Qdrant Cloud.
*   **Qdrant Cluster URL**: The URL of your Qdrant Cloud instance.

## 2. Environment Setup

1.  **Clone the Repository (if applicable)**:
    ```bash
    git clone <repository_url>
    cd physical-ai-book # or the directory containing the pipeline
    ```

2.  **Create a Virtual Environment**:
    It is highly recommended to use a virtual environment to manage dependencies.
    ```bash
    python -m venv .venv
    # On Windows
    .venv\Scripts\activate
    # On macOS/Linux
    source .venv/bin/activate
    ```

3.  **Install Dependencies**:
    Install the required Python packages. (A `requirements.txt` will be generated during implementation).
    ```bash
    pip install cohere qdrant-client beautifulsoup4 requests "langchain_text_splitters"
    ```

## 3. Configuration

The pipeline requires several environment variables for secure access to external services. Create a `.env` file in the root of your project (or wherever your main script is located) and populate it with the following:

```
COHERE_API_KEY="your_cohere_api_key_here"
QDRANT_API_KEY="your_qdrant_api_key_here"
QDRANT_CLUSTER_URL="your_qdrant_cluster_url_here"
DOCUSAURUS_BASE_URL="https://your-docusaurus-book-url.com" # e.g., https://physical-ai-book.com
QDRANT_COLLECTION_NAME="physical-ai-book-rag" # Or your desired collection name
```
**Note**: Replace placeholder values with your actual credentials and URLs.

## 4. Running the Ingestion Pipeline

1.  **Activate your virtual environment** (if not already active).

2.  **Execute the main script**:
    Once the `cli` module is implemented, you would run it like this (assuming `main.py` is the entry point):

    ```bash
    python src/cli/main.py ingest
    ```
    *(Further command-line arguments for specific configurations like `update` mode will be detailed in the `cli` documentation.)*

## 5. Verification

After the script completes:

1.  Log in to your Qdrant Cloud dashboard.
2.  Verify that a collection with the specified `QDRANT_COLLECTION_NAME` has been created.
3.  Check that the collection contains points (vectors) with the expected metadata (`source_url`, `module`, `section`, etc.).
4.  Optionally, perform a simple search in Qdrant to ensure relevance.
