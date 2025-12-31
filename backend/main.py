import sys
import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging

# Add the parent directory to Python path to import from src
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

try:
    from src.rag_agent import service as rag_agent_service
    logger.info("Successfully imported rag_agent_service")
except ImportError as e:
    logger.error(f"Failed to import rag_agent_service: {e}")
    raise

app = FastAPI()

origins = [
    "http://localhost:3000",  # For local development of Docusaurus
    "http://localhost:3001",  # Additional local development port
    "https://physical-ai-and-humanoid-robots.github.io",  # Deployed Docusaurus site
    "https://*.vercel.app",   # Vercel deployments
    "https://*.netlify.app",  # Netlify deployments
    "https://*.pages.dev",    # Cloudflare Pages
    "https://*.github.io",    # GitHub Pages
    "*"  # Allow all origins during development (be careful in production)
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

try:
    app.include_router(rag_agent_service.router, prefix="/agent", tags=["RAG Agent"])
    logger.info("Successfully included rag_agent_service router")
except Exception as e:
    logger.error(f"Failed to include rag_agent_service router: {e}")
    raise

@app.get("/")
async def root():
    return {"message": "RAG pipeline backend is running."}

