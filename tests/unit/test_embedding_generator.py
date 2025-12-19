import pytest
from unittest.mock import MagicMock, patch
import cohere
# from cohere import CohereError # Removed direct import
from src.embedding.embedding_generator import EmbeddingGenerator
from src.embedding.cohere_client import CohereClientManager
from src.utils.env_loader import load_env_variables

# Mock the environment loading to control API key behavior
@pytest.fixture(autouse=True)
def mock_env_loader():
    with patch('src.utils.env_loader.load_env_variables') as mock_load:
        with patch('src.utils.env_loader.get_env_variable') as mock_get:
            mock_get.return_value = "dummy_cohere_api_key" # Provide a dummy API key
            yield mock_load, mock_get

@pytest.fixture
def mock_cohere_client():
    """Mocks the cohere.Client instance."""
    mock = MagicMock(spec=cohere.Client)
    return mock

@pytest.fixture
def mock_cohere_client_manager(mock_cohere_client):
    """Mocks CohereClientManager to return our mocked cohere.Client."""
    manager = MagicMock(spec=CohereClientManager)
    manager.get_client.return_value = mock_cohere_client
    return manager

@pytest.fixture
def embedding_generator(mock_cohere_client_manager):
    """Provides an EmbeddingGenerator instance with mocked dependencies."""
    # Temporarily replace CohereClientManager with our mock
    with patch('src.embedding.embedding_generator.CohereClientManager', return_value=mock_cohere_client_manager):
        generator = EmbeddingGenerator()
        return generator

def test_embedding_generator_init(embedding_generator, mock_cohere_client_manager):
    mock_cohere_client_manager.get_client.assert_called_once()
    assert embedding_generator.model_name == "embed-english-v3.0"

def test_generate_embeddings_success(embedding_generator, mock_cohere_client):
    test_texts = ["hello world", "test sentence"]
    mock_embeddings_response = MagicMock()
    mock_embeddings_response.embeddings = [[0.1, 0.2], [0.3, 0.4]]
    mock_cohere_client.embed.return_value = mock_embeddings_response

    embeddings = embedding_generator.generate_embeddings(test_texts)

    mock_cohere_client.embed.assert_called_once_with(
        texts=test_texts,
        model="embed-english-v3.0",
        input_type="search_document"
    )
    assert embeddings == [[0.1, 0.2], [0.3, 0.4]]

def test_generate_embeddings_empty_texts(embedding_generator, mock_cohere_client):
    embeddings = embedding_generator.generate_embeddings([])
    mock_cohere_client.embed.assert_not_called()
    assert embeddings == []

def test_generate_embeddings_cohere_error(embedding_generator, mock_cohere_client):
    test_texts = ["error text"]
    mock_cohere_client.embed.side_effect = cohere.CohereError("API error") # Use cohere.CohereError

    with pytest.raises(cohere.CohereError, match="API error"):
        embedding_generator.generate_embeddings(test_texts)
    
    mock_cohere_client.embed.assert_called_once()

def test_embedding_generator_custom_model_name(mock_cohere_client_manager):
    with patch('src.embedding.embedding_generator.CohereClientManager', return_value=mock_cohere_client_manager):
        generator = EmbeddingGenerator(model_name="embed-multilingual-v3.0")
        assert generator.model_name == "embed-multilingual-v3.0"
        
        test_texts = ["custom model test"]
        mock_embeddings_response = MagicMock()
        mock_embeddings_response.embeddings = [[0.5, 0.6]]
        mock_cohere_client_manager.get_client().embed.return_value = mock_embeddings_response

        embeddings = generator.generate_embeddings(test_texts)
        mock_cohere_client_manager.get_client().embed.assert_called_once_with(
            texts=test_texts,
            model="embed-multilingual-v3.0",
            input_type="search_document"
        )
        assert embeddings == [[0.5, 0.6]]
