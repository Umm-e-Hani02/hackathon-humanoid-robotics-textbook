import pytest
import os
import time
from src.cli.main import ingest_command
from src.storage.qdrant_client import QdrantManager
from src.utils.env_loader import load_env_variables, get_env_variable
from unittest.mock import MagicMock

# Load environment variables once for the module
load_env_variables()

@pytest.fixture(scope="module")
def live_docusaurus_base_url():
    """
    Returns the live Docusaurus base URL from environment variables.
    Skips tests if not set.
    """
    url = get_env_variable("DOCUSAURUS_BASE_URL", None)
    if url is None:
        pytest.skip("DOCUSAURUS_BASE_URL not set in .env. Skipping E2E tests.")
    return url

@pytest.fixture(scope="module")
def qdrant_test_collection_name():
    """
    Returns the Qdrant collection name for E2E tests.
    """
    return get_env_variable("QDRANT_COLLECTION_NAME", "physical-ai-book-e2e-test")

@pytest.fixture(scope="module")
def qdrant_e2e_manager(qdrant_test_collection_name):
    """
    Provides a QdrantManager instance for E2E tests and ensures cleanup.
    """
    # Ensure API keys are available
    get_env_variable("QDRANT_API_KEY")
    get_env_variable("QDRANT_CLUSTER_URL")

    manager = QdrantManager()
    manager.collection_name = qdrant_test_collection_name
    
    # Ensure a clean slate before tests
    manager.delete_collection()
    time.sleep(1) # Give Qdrant a moment to process deletion

    yield manager

    # Clean up after tests
    print(f"\n[E2E Teardown] Deleting Qdrant collection '{manager.collection_name}'...")
    manager.delete_collection()

def test_e2e_full_ingestion_pipeline(live_docusaurus_base_url, qdrant_e2e_manager):
    """
    Runs the full ingestion pipeline end-to-end and verifies Qdrant population.
    """
    print(f"\n[E2E Test] Starting full ingestion pipeline for {live_docusaurus_base_url}")
    print(f"[E2E Test] Target Qdrant collection: {qdrant_e2e_manager.collection_name}")

    # Ensure Cohere API key is set
    get_env_variable("COHERE_API_KEY")

    # Prepare mock args for the CLI command
    mock_cli_args = MagicMock()
    mock_cli_args.base_url = live_docusaurus_base_url
    mock_cli_args.command = "ingest"

    # Execute the ingestion command
    ingest_command(mock_cli_args)

    # Allow some time for Qdrant to index if there's any eventual consistency delay
    time.sleep(5)

    # Verify Qdrant collection state
    collection_info = qdrant_e2e_manager.get_collection_info()
    assert collection_info is not None, "Qdrant collection was not created or found."
    assert collection_info.vectors_config.size == 1024 # Assuming Cohere embed-english-v3.0 default

    # Verify points count - should be greater than 0
    # The exact number depends on the Docusaurus site and chunking, so we just check > 0
    count_result = qdrant_e2e_manager.client.count(
        collection_name=qdrant_e2e_manager.collection_name,
        exact=True
    )
    print(f"[E2E Test] Qdrant collection contains {count_result.count} points.")
    assert count_result.count > 0, "No points were indexed in Qdrant collection."

    # Optional: Perform a simple search to ensure data is retrievable
    # This requires a dummy query embedding, so we mock Cohere to generate one.
    print("[E2E Test] Performing a sample search query to verify retrieval...")
    
    # Mock Cohere to return a dummy embedding for the search query
    with patch('src.embedding.embedding_generator.CohereClientManager.get_client') as mock_get_cohere_client:
        mock_cohere = MagicMock()
        mock_cohere.embed.return_value = MagicMock(embeddings=[[0.5]*1024])
        mock_get_cohere_client.return_value = mock_cohere

        search_results = qdrant_e2e_manager.client.search(
            collection_name=qdrant_e2e_manager.collection_name,
            query_vector=[0.5]*1024, # Use the dummy query embedding
            limit=3,
            with_payload=True,
        )
        print(f"[E2E Test] Sample search found {len(search_results)} results.")
        assert len(search_results) > 0, "Sample search query returned no results."
        assert "text_content" in search_results[0].payload, "Search result payload missing 'text_content'."
        assert "source_url" in search_results[0].payload, "Search result payload missing 'source_url'."
        print(f"[E2E Test] First search result payload: {search_results[0].payload}")

    print("[E2E Test] E2E ingestion pipeline test completed successfully.")
