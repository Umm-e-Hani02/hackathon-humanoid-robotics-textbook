import pytest
from unittest.mock import patch, MagicMock
import requests
import cohere
from src.cli.main import ingest_command
from src.storage.vector_store_manager import VectorStoreManager
from qdrant_client import models

# Fixture to mock environment variables for the entire test module
@pytest.fixture(autouse=True)
def mock_env_vars():
    with patch('src.utils.env_loader.load_env_variables') as mock_load:
        with patch('src.utils.env_loader.get_env_variable') as mock_get:
            mock_get.side_effect = lambda key, default=None: {
                "DOCUSAURUS_BASE_URL": "http://mockdocusaurus.com/",
                "COHERE_API_KEY": "mock_cohere_key",
                "QDRANT_API_KEY": "mock_qdrant_key",
                "QDRANT_CLUSTER_URL": "http://mock_qdrant_url",
                "QDRANT_COLLECTION_NAME": "mock_collection"
            }.get(key, default)
            yield mock_load, mock_get

# Fixture to mock requests.get for web crawling and content fetching
@pytest.fixture
def mock_requests_get():
    with patch('requests.get') as mock_get:
        yield mock_get

# Fixture to mock the Cohere client for embedding generation
@pytest.fixture
def mock_cohere_client_embed():
    with patch('cohere.Client.embed') as mock_embed:
        yield mock_embed

# Fixture to mock the Qdrant client's upsert method
@pytest.fixture
def mock_qdrant_client_upsert():
    with patch('qdrant_client.QdrantClient.upsert') as mock_upsert:
        yield mock_upsert

# Fixture to mock the QdrantManager and ensure create_collection is called
@pytest.fixture
def mock_qdrant_manager_create_collection():
    with patch('src.storage.qdrant_client.QdrantManager.create_collection') as mock_create:
        yield mock_create


def test_ingestion_pipeline_success(
    mock_requests_get, 
    mock_cohere_client_embed, 
    mock_qdrant_client_upsert,
    mock_qdrant_manager_create_collection
):
    # Mock crawler responses
    mock_requests_get.side_effect = [
        # Response for the base URL to find links
        MagicMock(status_code=200, raise_for_status=lambda: None, text="""
            <html><body>
                <a href="http://mockdocusaurus.com/docs/page1">Page 1</a>
                <a href="http://mockdocusaurus.com/docs/page2">Page 2</a>
            </body></html>
        """),
        # Response for Page 1
        MagicMock(status_code=200, raise_for_status=lambda: None, text="""
            <html><body><main><p>Content of page 1.</p></main></body></html>
        """),
        # Response for Page 2
        MagicMock(status_code=200, raise_for_status=lambda: None, text="""
            <html><body><main><h1>Heading 2</h1><p>Content of page 2.</p></main></body></html>
        """),
    ]

    # Mock Cohere embedding response
    mock_embed_response = MagicMock()
    mock_embed_response.embeddings = [
        [0.1]*1024, # Embedding for chunk 1 of page 1
        [0.2]*1024, # Embedding for chunk 1 of page 2
    ]
    mock_cohere_client_embed.return_value = mock_embed_response

    # Prepare mock args for the CLI command
    mock_cli_args = MagicMock()
    mock_cli_args.base_url = None # Use env var
    mock_cli_args.command = "ingest"

    # Execute the ingestion command
    ingest_command(mock_cli_args)

    # Assertions
    # Check that the crawler was called for base URL and discovered pages
    assert mock_requests_get.call_count == 3
    mock_requests_get.assert_any_call("http://mockdocusaurus.com/", timeout=5)
    mock_requests_get.assert_any_call("http://mockdocusaurus.com/docs/page1", timeout=10)
    mock_requests_get.assert_any_call("http://mockdocusaurus.com/docs/page2", timeout=10)

    # Check that embeddings were generated
    mock_cohere_client_embed.assert_called_once()
    # The actual texts passed to embed will depend on chunking, text cleaning etc.
    # So, we check that it was called with *some* list of texts.
    assert len(mock_cohere_client_embed.call_args[1]['texts']) == 2 # Assuming 2 chunks are generated

    # Check that Qdrant upsert was called with the correct data
    mock_qdrant_client_upsert.assert_called_once()
    upsert_call_args = mock_qdrant_client_upsert.call_args[1]
    
    assert upsert_call_args['collection_name'] == "mock_collection"
    assert upsert_call_args['wait'] is True
    assert len(upsert_call_args['points']) == 2 # 2 embeddings/points

    # Validate content of the points
    # Page 1 chunk
    point1 = upsert_call_args['points'][0]
    assert point1.vector == [0.1]*1024
    assert point1.payload['source_url'] == "http://mockdocusaurus.com/docs/page1"
    assert "content of page 1" in point1.payload['text_content'].lower()
    assert point1.payload['module'] == "Docs"
    assert point1.payload['chapter'] == "Page1"
    assert 'id' in point1.payload

    # Page 2 chunk
    point2 = upsert_call_args['points'][1]
    assert point2.vector == [0.2]*1024
    assert point2.payload['source_url'] == "http://mockdocusaurus.com/docs/page2"
    assert "content of page 2" in point2.payload['text_content'].lower()
    assert point2.payload['module'] == "Docs"
    assert point2.payload['chapter'] == "Page2"
    assert 'heading 2' in point2.payload['heading'].lower()
    assert 'id' in point2.payload

    # Ensure Qdrant collection creation was attempted
    mock_qdrant_manager_create_collection.assert_called_once_with(
        vector_size=1024, distance=models.Distance.COSINE
    )

def test_ingestion_pipeline_no_content_extracted(
    mock_requests_get, mock_cohere_client_embed, mock_qdrant_client_upsert
):
    mock_requests_get.side_effect = [
        MagicMock(status_code=200, raise_for_status=lambda: None, text="""
            <html><body><a href="http://mockdocusaurus.com/docs/empty">Empty Page</a></body></html>
        """),
        MagicMock(status_code=200, raise_for_status=lambda: None, text="""
            <html><body><header>No main content here</header></body></html>
        """), # Page with no extractable content
    ]

    mock_cli_args = MagicMock()
    mock_cli_args.base_url = None
    mock_cli_args.command = "ingest"

    ingest_command(mock_cli_args)

    assert mock_requests_get.call_count == 2
    mock_cohere_client_embed.assert_not_called() # No content, no embeddings
    mock_qdrant_client_upsert.assert_not_called() # No embeddings, no upsert


def test_ingestion_pipeline_http_error_during_fetch(
    mock_requests_get, mock_cohere_client_embed, mock_qdrant_client_upsert
):
    mock_requests_get.side_effect = [
        MagicMock(status_code=200, raise_for_status=lambda: None, text="""
            <html><body><a href="http://mockdocusaurus.com/docs/working">Working Page</a></body></html>
        """),
        requests.exceptions.RequestException("Simulated Network Error"), # Error fetching "Working Page"
    ]

    mock_embed_response = MagicMock()
    mock_embed_response.embeddings = [] # No embeddings since fetch failed
    mock_cohere_client_embed.return_value = mock_embed_response

    mock_cli_args = MagicMock()
    mock_cli_args.base_url = None
    mock_cli_args.command = "ingest"

    ingest_command(mock_cli_args)

    assert mock_requests_get.call_count == 2 # Base URL + failed page
    mock_cohere_client_embed.assert_not_called()
    mock_qdrant_client_upsert.assert_not_called()

def test_ingestion_pipeline_cohere_error(
    mock_requests_get, mock_cohere_client_embed, mock_qdrant_client_upsert
):
    mock_requests_get.side_effect = [
        MagicMock(status_code=200, raise_for_status=lambda: None, text="""
            <html><body><a href="http://mockdocusaurus.com/docs/page1">Page 1</a></body></html>
        """),
        MagicMock(status_code=200, raise_for_status=lambda: None, text="""
            <html><body><main><p>Content of page 1.</p></main></body></html>
        """),
    ]
    mock_cohere_client_embed.side_effect = cohere.CohereError("Simulated Cohere Error")

    mock_cli_args = MagicMock()
    mock_cli_args.base_url = None
    mock_cli_args.command = "ingest"

    ingest_command(mock_cli_args)

    assert mock_requests_get.call_count == 2
    mock_cohere_client_embed.assert_called_once()
    mock_qdrant_client_upsert.assert_not_called() # No embeddings, no upsert

