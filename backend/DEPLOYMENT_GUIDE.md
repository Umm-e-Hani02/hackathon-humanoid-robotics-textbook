# Deployment Guide for FastAPI RAG Backend

## Deploying to Render

1. Create a free account at [Render](https://render.com)

2. Create a new Web Service:
   - Connect your GitHub repository containing the backend code
   - Choose the backend directory
   - Runtime: Python
   - Build Command: `pip install -r requirements.txt`
   - Start Command: `uvicorn main:app --host=0.0.0.0 --port $PORT`

3. Environment Variables:
   - `QDRANT_URL`: Your Qdrant cluster URL
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_COLLECTION_NAME`: Your Qdrant collection name (default: physical-ai-book-rag)

4. The service will be deployed at a URL like: `https://your-service-name.onrender.com`

## Deploying to Heroku

1. Create a free account at [Heroku](https://heroku.com)

2. Install Heroku CLI and log in

3. Create a new app:
   ```bash
   heroku create your-app-name
   ```

4. Set environment variables:
   ```bash
   heroku config:set QDRANT_URL=your_qdrant_url
   heroku config:set QDRANT_API_KEY=your_qdrant_api_key
   heroku config:set COHERE_API_KEY=your_cohere_api_key
   heroku config:set QDRANT_COLLECTION_NAME=physical-ai-book-rag
   ```

5. Deploy:
   ```bash
   git add .
   git commit -m "Prepare for deployment"
   git push heroku main
   ```

## Deploying with Docker

1. Build the Docker image:
   ```bash
   docker build -t rag-backend .
   ```

2. Run the container (with environment variables):
   ```bash
   docker run -p 8000:8000 \
     -e QDRANT_URL=your_qdrant_url \
     -e QDRANT_API_KEY=your_qdrant_api_key \
     -e COHERE_API_KEY=your_cohere_api_key \
     -e QDRANT_COLLECTION_NAME=physical-ai-book-rag \
     rag-backend
   ```

## Testing the Deployment

Once deployed, you can test the API at:
- Root endpoint: `https://your-deployed-url/`
- Chat endpoint: `https://your-deployed-url/agent/chat` (POST request with {"text": "your query"})

## Important Notes

- Keep your API keys secure and never commit them to version control
- The deployed backend will connect to the same Qdrant database as your local setup
- Make sure your Qdrant cluster is accessible from the public internet
- The RAG logic, embeddings, and Qdrant data remain unchanged