import unittest
from unittest.mock import MagicMock, patch
from src.rag_agent.agent import RagAgent

class TestRagAgent(unittest.TestCase):
    @patch('src.rag_agent.agent.CohereClientManager')
    @patch('src.rag_agent.agent.QdrantManager')
    def test_retrieve_relevant_content(self, mock_qdrant_manager, mock_cohere_manager):
        # Arrange
        mock_cohere_client = MagicMock()
        mock_cohere_client.embed.return_value.embeddings = [[0.1] * 1024]
        mock_cohere_manager.return_value.get_client.return_value = mock_cohere_client

        mock_qdrant_client = MagicMock()
        mock_qdrant_client.search.return_value = [
            MagicMock(payload={'text': 'This is a relevant chunk.'}, score=0.9)
        ]
        mock_qdrant_manager.return_value.search.return_value = [
            MagicMock(payload={'text': 'This is a relevant chunk.'}, score=0.9)
        ]

        agent = RagAgent()
        query = "What is relevant?"

        # Act
        result = agent.retrieve(query)

        # Assert
        self.assertIn("relevant chunk", result)

    @patch('src.rag_agent.agent.CohereClientManager')
    @patch('src.rag_agent.agent.QdrantManager')
    def test_handle_out_of_scope_query(self, mock_qdrant_manager, mock_cohere_manager):
        # Arrange
        mock_cohere_client = MagicMock()
        mock_cohere_client.embed.return_value.embeddings = [[0.1] * 1024]
        mock_cohere_manager.return_value.get_client.return_value = mock_cohere_client

        mock_qdrant_client = MagicMock()
        mock_qdrant_client.search.return_value = []
        mock_qdrant_manager.return_value.search.return_value = []

        agent = RagAgent()
        query = "This is an out-of-scope query."

        # Act
        result = agent.retrieve(query)

        # Assert
        self.assertEqual("I couldn't find any relevant information in the book.", result)
