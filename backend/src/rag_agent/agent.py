import logging
import time
from src.storage.qdrant_client import QdrantManager
from src.embedding.cohere_client import CohereClientManager
import cohere

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RagAgent:
    def __init__(self):
        self.qdrant_manager = QdrantManager()
        self.cohere_manager = CohereClientManager()
        self.cohere_client = self.cohere_manager.get_client()
        self.system_prompt = self._get_system_prompt()

        # Configuration for rate limit handling
        self.max_retries = 3
        self.base_retry_delay = 2  # seconds
        self.min_query_length = 5  # Minimum length to trigger RAG pipeline

    def _get_system_prompt(self):
        return """
You are a helpful chatbot for the "Physical AI and Humanoid Robots" book.
Your goal is to answer user questions based on the content of the book.

Follow these rules strictly:
1. Respond directly to the user's question.
2. Use the provided book content (documents) to answer the question. Only use the content if it is relevant.
3. Do not include navigation text, page dumps, or unrelated content in your answer.
4. Keep your answers natural and concise.
5. If the answer is not found in the provided book content, say "I could not find an answer to that question in the book." Do not try to answer from your own knowledge.
"""

    def _is_trivial_query(self, query: str) -> bool:
        """
        Check if query is too short or trivial to warrant RAG processing.
        This helps avoid unnecessary API calls for greetings like "hi", "hello".
        """
        cleaned_query = query.strip()
        return len(cleaned_query) < self.min_query_length

    def _get_greeting_response(self, query: str) -> str:
        """
        Return a friendly response for trivial/greeting queries without RAG.
        """
        query_lower = query.strip().lower()

        greetings = ["hi", "hello", "hey", "greetings", "good morning", "good afternoon", "good evening"]
        if any(greeting in query_lower for greeting in greetings):
            return "Hello! I'm here to help you with questions about Physical AI and Humanoid Robots. Feel free to ask me anything about the book!"

        return "Hi there! Please ask me a specific question about Physical AI and Humanoid Robots, and I'll do my best to help you."

    def _embed_with_retry(self, query: str) -> list:
        """
        Generate embeddings with exponential backoff retry logic.
        Handles Cohere rate limits (HTTP 429) gracefully.

        Returns:
            list: Query embedding vector
        Raises:
            Exception: If all retries fail
        """
        for attempt in range(self.max_retries):
            try:
                logger.info(f"Generating embedding (attempt {attempt + 1}/{self.max_retries})")

                embedding = self.cohere_client.embed(
                    texts=[query],
                    model=self.cohere_manager.get_embedding_model(),
                    input_type="search_query"
                ).embeddings[0]

                logger.info("Embedding generated successfully")
                return embedding

            except cohere.errors.TooManyRequestsError as e:
                # Rate limit hit - implement exponential backoff
                if attempt < self.max_retries - 1:
                    delay = self.base_retry_delay * (attempt + 1)
                    logger.warning(
                        f"Rate limit hit (429). Retrying in {delay}s... "
                        f"(attempt {attempt + 1}/{self.max_retries})"
                    )
                    time.sleep(delay)
                else:
                    logger.error("Max retries reached for embedding generation")
                    raise

            except Exception as e:
                logger.error(f"Unexpected error during embedding: {e}")
                raise

    def retrieve(self, query: str, selected_text: str = None):
        """
        Main RAG retrieval method with optimizations and error handling.

        Features:
        - Skips RAG for trivial queries (< 5 chars)
        - Handles Cohere rate limits with retry logic
        - Supports selected text as additional context
        - Always returns valid response (never crashes)

        Args:
            query: User's question
            selected_text: Optional text selected by user for context
        """
        logger.info(f"Received query: '{query}' (length: {len(query.strip())})")
        if selected_text:
            logger.info(f"Selected text context provided (length: {len(selected_text)})")

        try:
            # OPTIMIZATION: Skip RAG for very short/trivial queries
            if self._is_trivial_query(query):
                logger.info("Query is trivial - returning direct greeting (no RAG)")
                return self._get_greeting_response(query)

            # RATE LIMIT PROTECTION: Generate embedding with retry logic
            try:
                query_embedding = self._embed_with_retry(query)
            except cohere.errors.TooManyRequestsError:
                logger.error("Rate limit exceeded after retries - returning fallback")
                return (
                    "I'm currently experiencing high load. Please try again in a moment. "
                    "In the meantime, feel free to explore the Physical AI and Humanoid Robots book!"
                )
            except Exception as e:
                logger.error(f"Embedding generation failed: {e}")
                return "I'm having trouble processing your question right now. Please try rephrasing or ask again shortly."

            # Search vector database
            search_result = self.qdrant_manager.search(query_vector=query_embedding)

            if not search_result:
                logger.info("No relevant information found in the book.")
                return "I could not find an answer to that question in the book."

            # Prepare documents for Cohere chat
            documents = [
                {
                    'title': point.payload.get('source_url', point.payload.get('source_file', 'Unknown')),
                    'text': point.payload['text']
                }
                for point in search_result
            ]

            # If selected text is provided, add it as the first document with highest priority
            if selected_text and selected_text.strip():
                documents.insert(0, {
                    'title': 'Selected Text (User Context)',
                    'text': selected_text.strip()
                })
                logger.info(f"Added selected text as priority context document")

            logger.info(f"Retrieved {len(documents)} relevant chunks from book")

            # RATE LIMIT PROTECTION: Generate response with retry logic
            try:
                # Modify system prompt if selected text is provided
                system_prompt = self.system_prompt
                if selected_text and selected_text.strip():
                    system_prompt = self.system_prompt + "\n\nIMPORTANT: The user has selected specific text from the book. This selected text is provided as the first document titled 'Selected Text (User Context)'. Prioritize this selected text when answering the user's question, as it represents the specific content they are asking about."

                response = self._generate_response_with_retry(query, documents, system_prompt)
                logger.info(f"Generated response: {response[:100]}...")
                return response

            except cohere.errors.TooManyRequestsError:
                logger.error("Rate limit hit during chat generation - returning context only")
                # Fallback: Return a simple response using retrieved context
                context_preview = documents[0]['text'][:200] if documents else ""
                return (
                    f"I found relevant information but I'm currently rate-limited. "
                    f"Here's what I found: {context_preview}... "
                    f"Please try again in a moment for a complete answer."
                )

        except Exception as e:
            # STABILITY: Catch any unexpected errors
            logger.error(f"Unexpected error in retrieve(): {e}", exc_info=True)
            return "An unexpected error occurred. Please try again later."

    def _generate_response_with_retry(self, query: str, documents: list, system_prompt: str = None) -> str:
        """
        Generate chat response with retry logic for rate limits.

        Args:
            query: User's question
            documents: List of relevant documents from RAG
            system_prompt: Optional custom system prompt (uses default if not provided)

        Returns:
            str: Generated response text
        Raises:
            Exception: If all retries fail
        """
        # Use provided system prompt or fall back to default
        prompt = system_prompt if system_prompt else self.system_prompt

        for attempt in range(self.max_retries):
            try:
                logger.info(f"Generating chat response (attempt {attempt + 1}/{self.max_retries})")

                response = self.cohere_client.chat(
                    model="command-r-08-2024",
                    message=query,
                    documents=documents,
                    preamble=prompt
                )

                return response.text

            except cohere.errors.TooManyRequestsError as e:
                if attempt < self.max_retries - 1:
                    delay = self.base_retry_delay * (attempt + 1)
                    logger.warning(
                        f"Rate limit hit during chat (429). Retrying in {delay}s... "
                        f"(attempt {attempt + 1}/{self.max_retries})"
                    )
                    time.sleep(delay)
                else:
                    logger.error("Max retries reached for chat generation")
                    raise

            except Exception as e:
                logger.error(f"Unexpected error during chat: {e}")
                raise
