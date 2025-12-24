import logging
from src.storage.qdrant_client import QdrantManager
from src.embedding.cohere_client import CohereClientManager

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RagAgent:
    def __init__(self):
        self.qdrant_manager = QdrantManager()
        self.cohere_manager = CohereClientManager()
        self.cohere_client = self.cohere_manager.get_client()
        self.system_prompt = self._get_system_prompt()

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

    def retrieve(self, query: str):
        logger.info(f"Received query: {query}")
        try:
            query_embedding = self.cohere_client.embed(
                texts=[query],
                model=self.cohere_manager.get_embedding_model(),
                input_type="search_query"
            ).embeddings[0]

            search_result = self.qdrant_manager.search(query_vector=query_embedding)

            if not search_result:
                logger.info("No relevant information found in the book.")
                return "I could not find an answer to that question in the book."

            documents = [{'title': point.payload['source_url'], 'text': point.payload['text']} for point in search_result]
            logger.info(f"Retrieved {len(documents)} chunks.")

            response = self.cohere_client.chat(
                model=self.cohere_manager.get_chat_model(),
                message=query,
                documents=documents,
                preamble=self.system_prompt
            )

            logger.info(f"Generated response: {response.text[:100]}...")
            return response.text
        except Exception as e:
            logger.error(f"An error occurred during retrieval: {e}", exc_info=True)
            return "An unexpected error occurred. Please try again later."
