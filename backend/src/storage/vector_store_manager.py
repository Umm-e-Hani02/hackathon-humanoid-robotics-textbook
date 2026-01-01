from typing import List, Dict, Any
from qdrant_client import models
from src.storage.qdrant_client import QdrantManager
from src.embedding.embedding_generator import EmbeddingGenerator # To get vector size

class VectorStoreManager:
    def __init__(self, collection_name: str = None):
        """
        Initializes the VectorStoreManager.

        Args:
            collection_name (str, optional): The name of the Qdrant collection to manage.
                                             If None, it will be loaded from environment variables.
        """
        self.qdrant_manager = QdrantManager()
        if collection_name:
            self.qdrant_manager.collection_name = collection_name
        self.collection_name = self.qdrant_manager.collection_name

        # Determine vector size for collection creation.
        # Assuming a default model dimension or fetching it.
        # For Cohere embed-english-v3.0, it's 1024. This should be consistent.
        self.vector_size = 1024 # Hardcoding for now, ideally derived from embedding model

        # Ensure the collection exists with the correct configuration
        self.qdrant_manager.create_collection(vector_size=self.vector_size, distance=models.Distance.COSINE)


    def upsert_vectors_batch(self,
                             vectors: List[List[float]],
                             payloads: List[Dict[str, Any]],
                             ids: List[str] = None):
        """
        Upserts (inserts or updates) vectors and their associated payloads into the Qdrant collection.

        Args:
            vectors (List[List[float]]): A list of vectors to upsert.
            payloads (List[Dict[str, Any]]): A list of metadata payloads, one for each vector.
            ids (List[str], optional): Optional list of unique IDs for the points. If None, Qdrant will generate them.
        """
        if not vectors or not payloads:
            print("No vectors or payloads provided for upsert.")
            return

        if len(vectors) != len(payloads):
            raise ValueError("Number of vectors and payloads must match.")
        if ids and len(ids) != len(vectors):
            raise ValueError("Number of IDs must match number of vectors if provided.")

        points = []
        for i, vector in enumerate(vectors):
            point_id = ids[i] if ids else None # Use provided ID or let Qdrant generate
            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=vector,
                    payload=payloads[i]
                )
            )
        
        operation_info = self.qdrant_manager.client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points
        )
        print(f"Upsert operation completed: {operation_info}")

if __name__ == "__main__":
    # Example Usage:
    # Ensure .env has QDRANT_CLUSTER_URL, QDRANT_API_KEY, COHERE_API_KEY
    # and QDRANT_COLLECTION_NAME (optional) set.
    
    # It's good practice to ensure environment variables are loaded here for standalone execution
    from src.utils.env_loader import load_env_variables
    load_env_variables()

    print("Initializing VectorStoreManager...")
    manager = VectorStoreManager(collection_name="test_collection_physical_ai")

    # Generate some dummy data
    dummy_texts = [
        "This is a test sentence for embedding.",
        "Another sentence to test the embedding and storage.",
        "A third short text to process.",
        "The quick brown fox jumps over the lazy dog."
    ]
    dummy_metadata = [
        {"source_url": "http://example.com/page1", "module": "Test", "section": "Intro"},
        {"source_url": "http://example.com/page2", "module": "Test", "section": "Advanced"},
        {"source_url": "http://example.com/page1", "module": "Test", "section": "Conclusion"},
        {"source_url": "http://example.com/page3", "module": "Animals", "section": "Canine"}
    ]

    # Use EmbeddingGenerator to get actual-like vectors
    try:
        embedding_gen = EmbeddingGenerator()
        dummy_vectors = embedding_gen.generate_embeddings(dummy_texts)
        print(f"Generated {len(dummy_vectors)} dummy vectors.")

        # Upsert the vectors and metadata
        print("\nUpserting dummy data to Qdrant...")
        manager.upsert_vectors_batch(vectors=dummy_vectors, payloads=dummy_metadata)
        print("Dummy data upserted.")

        # Verify by checking collection count
        count_result = manager.qdrant_manager.client.count(collection_name=manager.collection_name, exact=True)
        print(f"Current point count in collection '{manager.collection_name}': {count_result.count}")

    except Exception as e:
        print(f"Error during example usage: {e}")
    finally:
        # Clean up the test collection
        print(f"\nDeleting test collection '{manager.collection_name}'...")
        manager.qdrant_manager.delete_collection()
        print("Test collection deleted.")
