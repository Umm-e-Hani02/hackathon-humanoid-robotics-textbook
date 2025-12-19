import pytest
from unittest.mock import MagicMock, patch
from qdrant_client import models
from src.storage.vector_store_manager import VectorStoreManager
from src.storage.qdrant_client import QdrantManager
from src.embedding.embedding_generator import EmbeddingGenerator

# Mock environment variables for all tests
@pytest.fixture(autouse=True)
def mock_env_vars():
    with patch('src.utils.env_loader.load_env_variables') as mock_load:
        with patch('src.utils.env_loader.get_env_variable') as mock_get:
            mock_get.side_effect = lambda key, default=None: {
                "QDRANT_CLUSTER_URL": "http://localhost:6333",
                "QDRANT_API_KEY": "dummy_qdrant_key",
                "QDRANT_COLLECTION_NAME": "test_collection",
                "COHERE_API_KEY": "dummy_cohere_key"
            }.get(key, default)
            yield mock_load, mock_get

@pytest.fixture
def mock_qdrant_client():
    """Mocks the QdrantClient instance."""
    mock = MagicMock()
    mock.collection_exists.return_value = False # Assume collection does not exist initially
    mock.recreate_collection.return_value = None
    mock.upsert.return_value = MagicMock(status=models.UpdateStatus.COMPLETED)
    mock.count.return_value = MagicMock(count=0) # Default count to 0
    return mock

@pytest.fixture
def mock_qdrant_manager(mock_qdrant_client):
    """Mocks QdrantManager to use our mocked QdrantClient."""
    manager = MagicMock(spec=QdrantManager)
    manager.client = mock_qdrant_client
    manager.collection_name = "test_collection"
    # These attributes are on the client, not the manager directly.
    # The actual QdrantManager calls manager.client.method, so this mock needs to reflect that.
    manager.client.collection_exists.return_value = False
    manager.client.recreate_collection.return_value = None
    manager.client.delete_collection.return_value = None
    manager.client.get_collection.return_value = MagicMock(config=MagicMock(vectors_config=MagicMock(size=1024, distance=models.Distance.COSINE)))

    return manager

@pytest.fixture
def mock_embedding_generator():
    """Mocks EmbeddingGenerator to return dummy embeddings."""
    mock = MagicMock(spec=EmbeddingGenerator)
    mock.generate_embeddings.return_value = [[0.1]*1024, [0.2]*1024]
    return mock

@pytest.fixture
def vector_store_manager(mock_qdrant_manager):
    """Provides a VectorStoreManager instance with mocked QdrantManager."""
    with patch('src.storage.vector_store_manager.QdrantManager', return_value=mock_qdrant_manager):
        manager = VectorStoreManager()
        return manager

def test_vector_store_manager_init(vector_store_manager, mock_qdrant_manager):
    # Now check calls on the mock_qdrant_manager.client
    mock_qdrant_manager.client.collection_exists.assert_called_once_with(collection_name="test_collection")
    mock_qdrant_manager.client.recreate_collection.assert_called_once_with(
        collection_name="test_collection",
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
    )
    assert vector_store_manager.collection_name == "test_collection"
    assert vector_store_manager.vector_size == 1024

def test_upsert_vectors_batch_success(vector_store_manager, mock_qdrant_client):
    vectors = [[0.1]*1024, [0.2]*1024]
    payloads = [
        {"source_url": "url1", "text_content": "text1", "id": "hash1"},
        {"source_url": "url2", "text_content": "text2", "id": "hash2"}
    ]
    ids = ["hash1", "hash2"]

    vector_store_manager.upsert_vectors_batch(vectors, payloads, ids)

    mock_qdrant_client.upsert.assert_called_once()
    args, kwargs = mock_qdrant_client.upsert.call_args
    assert kwargs['collection_name'] == "test_collection"
    assert kwargs['wait'] is True
    
    points = kwargs['points']
    assert len(points) == 2
    assert points[0].id == "hash1"
    assert points[0].vector == vectors[0]
    assert points[0].payload == payloads[0]
    assert points[1].id == "hash2"
    assert points[1].vector == vectors[1]
    assert points[1].payload == payloads[1]

def test_upsert_vectors_batch_empty_input(vector_store_manager, mock_qdrant_client):
    vector_store_manager.upsert_vectors_batch([], [])
    mock_qdrant_client.upsert.assert_not_called()

def test_upsert_vectors_batch_mismatched_lengths(vector_store_manager):
    vectors = [[0.1]*1024]
    payloads = [{"source_url": "url1", "text_content": "text1"}]
    ids = ["hash1", "hash2"] # Mismatched

    with pytest.raises(ValueError, match="Number of IDs must match number of vectors if provided."):
        vector_store_manager.upsert_vectors_batch(vectors, payloads, ids)
    
    vectors = [[0.1]*1024]
    payloads = [] # Mismatched
    with pytest.raises(ValueError, match="Number of vectors and payloads must match."):
        vector_store_manager.upsert_vectors_batch(vectors, payloads)

def test_upsert_vectors_batch_qdrant_error(vector_store_manager, mock_qdrant_client):
    vectors = [[0.1]*1024]
    payloads = [{"source_url": "url1", "text_content": "text1", "id": "hash1"}]
    mock_qdrant_client.upsert.side_effect = Exception("Qdrant connection error")

    with patch('builtins.print') as mock_print: # Capture print statements
        vector_store_manager.upsert_vectors_batch(vectors, payloads)
        # We expect a print for the operation info, even if it failed internally in the mock
        mock_print.assert_called_with(f"Upsert operation completed: {mock_qdrant_client.upsert.return_value}")

def test_upsert_vectors_batch_no_ids_provided(vector_store_manager, mock_qdrant_client):
    vectors = [[0.1]*1024, [0.2]*1024]
    payloads = [
        {"source_url": "url1", "text_content": "text1"},
        {"source_url": "url2", "text_content": "text2"}
    ]

    vector_store_manager.upsert_vectors_batch(vectors, payloads)
    
    mock_qdrant_client.upsert.assert_called_once()
    args, kwargs = mock_qdrant_client.upsert.call_args
    points = kwargs['points']
    assert len(points) == 2
    assert points[0].id is None # Qdrant will generate
    assert points[1].id is None