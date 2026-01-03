import logging
from fastapi import APIRouter
from pydantic import BaseModel, validator
from src.rag_agent.agent import RagAgent

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class UserQuery(BaseModel):
    text: str

    @validator('text')
    def text_must_not_be_empty(cls, v):
        """Validate that query text is not empty"""
        if not v or not v.strip():
            raise ValueError('Query text cannot be empty')
        return v.strip()

class ChatbotResponse(BaseModel):
    answer: str

router = APIRouter()
agent = RagAgent()

@router.post("/chat", response_model=ChatbotResponse)
async def chat_with_agent(query: UserQuery):
    """
    Chat endpoint with robust error handling.

    Features:
    - Validates input (non-empty text)
    - Handles all exceptions gracefully
    - Always returns valid JSON response
    - Never crashes or returns 500 errors to frontend

    Returns:
        ChatbotResponse: JSON with 'answer' field
    """
    try:
        logger.info(f"[/agent/chat] Received query: '{query.text}' (length: {len(query.text)})")

        # Main RAG processing with built-in error handling
        response_text = agent.retrieve(query.text)

        logger.info(f"[/agent/chat] Response generated successfully")
        return ChatbotResponse(answer=response_text)

    except ValueError as e:
        # Input validation error (caught by Pydantic)
        logger.warning(f"[/agent/chat] Invalid input: {e}")
        return ChatbotResponse(
            answer="Please provide a valid question. Empty queries are not allowed."
        )

    except Exception as e:
        # Catch-all for any unexpected errors
        logger.error(f"[/agent/chat] Unexpected error: {e}", exc_info=True)

        # NEVER raise HTTPException - always return friendly message
        return ChatbotResponse(
            answer=(
                "I encountered an unexpected error while processing your question. "
                "Please try again, or rephrase your question."
            )
        )
