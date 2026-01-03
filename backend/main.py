import sys
import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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

# Try to import and include the router
import sys
import os

# Add the current directory (backend/) to sys.path so 'src' can be found locally
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

try:
    # Since we moved src into backend/, it should be discoverable locally
    from src.rag_agent import service as rag_agent_service
    app.include_router(rag_agent_service.router, prefix="/agent", tags=["RAG Agent"])
    logger.info("Successfully included rag_agent_service router")
except ImportError as e:
    logger.error(f"Failed to import rag_agent_service: {e}")
    # Try alternative import if running in a different context
    try:
        from backend.src.rag_agent import service as rag_agent_service
        app.include_router(rag_agent_service.router, prefix="/agent", tags=["RAG Agent"])
        logger.info("Successfully included rag_agent_service router via backend.src")
    except ImportError as e2:
        logger.error(f"Failed to import via backend.src: {e2}")
        logger.error("The RAG agent functionality will not be available")
except Exception as e:
    logger.error(f"Failed to include rag_agent_service router: {e}")
    logger.error("The RAG agent functionality will not be available")

@app.get("/")
async def root():
    return {"message": "RAG pipeline backend is running."}

