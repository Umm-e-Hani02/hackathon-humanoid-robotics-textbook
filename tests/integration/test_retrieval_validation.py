"""
Integration tests for retrieval validation of the RAG pipeline.
"""
import pytest
import logging
from src.storage.qdrant_client import QdrantManager
from src.embedding.cohere_client import CohereClientManager
from src.embedding.embedding_generator import EmbeddingGenerator
from tests.unit.sample_queries import SAMPLE_QUERIES

# Configure logging for better visibility during test execution
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

@pytest.fixture(scope="module")
def rag_clients():
    """
    Pytest fixture to initialize and provide QdrantManager and CohereClientManager instances.
    """
    qdrant_manager = QdrantManager()
    cohere_manager = CohereClientManager()
    yield qdrant_manager, cohere_manager

def test_qdrant_connection():
    """
    Verifies that the QdrantManager can connect to the Qdrant instance
    and retrieve collection information.
    """
    manager = QdrantManager()
    collection_info = manager.get_collection_info()
    assert collection_info is not None, "Failed to connect to Qdrant or retrieve collection info."
    print(f"\nSuccessfully connected to Qdrant. Collection Info: {collection_info}")

def test_cohere_client_init():
    """
    Verifies that the CohereClientManager can be initialized and provides a client.
    """
    manager = CohereClientManager()
    cohere_client = manager.get_client()
    assert cohere_client is not None, "Failed to initialize CohereClientManager or get client."
    print(f"\nSuccessfully initialized CohereClientManager. Client is: {cohere_client is not None}")

@pytest.mark.parametrize("query_text", SAMPLE_QUERIES)
def test_similarity_search_parametrized_query(rag_clients, query_text):
    """
    Performs a similarity search using a parametrized sample query and validates the structure and metadata
    of the retrieved results. Logs the query, retrieved chunk IDs, relevance scores, and validation outcomes.
    """
    qdrant_manager, cohere_manager = rag_clients

    logging.info(f"Performing similarity search for query: '{query_text}'")

    assert qdrant_manager.client.collection_exists(collection_name=qdrant_manager.collection_name), \
        f"Collection '{qdrant_manager.collection_name}' does not exist. Please run ingestion first."

    embedding_generator = EmbeddingGenerator()
    query_embedding = embedding_generator.generate_embeddings([query_text])[0]
    assert query_embedding is not None, "Failed to generate embedding for the query."

    search_result = qdrant_manager.client.search(
        collection_name=qdrant_manager.collection_name,
        query_vector=query_embedding,
        limit=5,
        with_payload=True
    )

    logging.info(f"Query: '{query_text}', Retrieved {len(search_result)} results.")
    retrieved_ids = []
    for scored_point in search_result:
        retrieved_ids.append(scored_point.id)
        logging.info(f"  - ID: {scored_point.id}, Score: {scored_point.score}, Payload: {scored_point.payload}")

        assert hasattr(scored_point, "id")
        assert hasattr(scored_point, "payload")
        assert hasattr(scored_point, "score")
        assert isinstance(scored_point.id, (int, str))
        assert isinstance(scored_point.score, float)
        assert isinstance(scored_point.payload, dict)

        assert "url" in scored_point.payload, f"Payload missing 'url' for point ID {scored_point.id}"
        assert "chapter" in scored_point.payload, f"Payload missing 'chapter' for point ID {scored_point.id}"
        assert "section" in scored_point.payload, f"Payload missing 'section' for point ID {scored_point.id}"
        assert isinstance(scored_point.payload["url"], str)
        assert isinstance(scored_point.payload["chapter"], str)
        assert isinstance(scored_point.payload["section"], str)
    
    if not search_result:
        logging.warning(f"No results returned for query: '{query_text}'. Ensure data is ingested into the collection.")
    
    logging.info(f"Validation successful for query: '{query_text}'.")