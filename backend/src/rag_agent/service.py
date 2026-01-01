import logging
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from src.rag_agent.agent import RagAgent

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class UserQuery(BaseModel):
    text: str

class ChatbotResponse(BaseModel):
    answer: str

router = APIRouter()
agent = RagAgent()

@router.post("/chat", response_model=ChatbotResponse)
async def chat_with_agent(query: UserQuery):
    try:
        logger.info(f"Received query for /chat endpoint: {query.text}")
        response_text = agent.retrieve(query.text)
        return ChatbotResponse(answer=response_text)
    except Exception as e:
        logger.error(f"Error processing query: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))
