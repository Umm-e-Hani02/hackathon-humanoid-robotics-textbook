from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.rag_agent import service as rag_agent_service

app = FastAPI()

origins = [
    "http://localhost:3000",  # For local development of Docusaurus
    "https://physical-ai-and-humanoid-robots.github.io",  # Deployed Docusaurus site
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(rag_agent_service.router, prefix="/agent", tags=["RAG Agent"])

@app.get("/")
async def root():
    return {"message": "RAG pipeline backend is running."}

