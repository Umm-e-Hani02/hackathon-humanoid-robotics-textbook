from qdrant_client import QdrantClient, models
from src.utils.env_loader import get_env_variable, load_env_variables

class QdrantManager:
    def __init__(self):
        load_env_variables()  # Ensure environment variables are loaded
        self.qdrant_url = get_env_variable("QDRANT_URL")
        self.qdrant_api_key = get_env_variable("QDRANT_API_KEY")
        self.collection_name = get_env_variable("QDRANT_COLLECTION_NAME", "physical-ai-book-rag")
        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
        )

    def create_collection(self, vector_size: int, distance: models.Distance = models.Distance.COSINE):
        """
        Creates a new Qdrant collection if it does not already exist.

        Args:
            vector_size (int): The dimension of the vectors to be stored.
            distance (models.Distance): The distance metric for the collection (e.g., COSINE, EUCLID, DOT).
        """
        if not self.client.collection_exists(collection_name=self.collection_name):
            self.client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=vector_size, distance=distance),
            )
            print(f"Collection '{self.collection_name}' created with vector size {vector_size} and distance {distance}.")
        else:
            print(f"Collection '{self.collection_name}' already exists.")

    def get_collection_info(self):
        """
        Retrieves information about the current collection.

        Returns:
            models.CollectionInfo or None: Information about the collection, or None if it does not exist.
        """
        if self.client.collection_exists(collection_name=self.collection_name):
            return self.client.get_collection(collection_name=self.collection_name).config
        print(f"Collection '{self.collection_name}' does not exist.")
        return None

    def delete_collection(self):
        """
        Deletes the Qdrant collection if it exists.
        """
        if self.client.collection_exists(collection_name=self.collection_name):
            self.client.delete_collection(collection_name=self.collection_name)
            print(f"Collection '{self.collection_name}' deleted.")
        else:
            print(f"Collection '{self.collection_name}' does not exist.")

if __name__ == "__main__":
    # This is a placeholder for local testing
    # In a real scenario, ensure your .env has QDRANT_CLUSTER_URL and QDRANT_API_KEY set
    # and potentially QDRANT_COLLECTION_NAME
    print("Initializing QdrantManager...")
    manager = QdrantManager()

    # Example Usage:
    # 1. Delete existing collection if any
    print("\nAttempting to delete collection...")
    manager.delete_collection()

    # 2. Create a new collection (e.g., for Cohere embed-english-v3.0 which has 1024 dimensions)
    print("\nAttempting to create collection...")
    COHERE_EMBEDDING_DIMENSION = 1024 # Example dimension for a Cohere model
    manager.create_collection(vector_size=COHERE_EMBEDDING_DIMENSION)

    # 3. Get collection info
    print("\nAttempting to get collection info...")
    info = manager.get_collection_info()
    if info:
        print(f"Collection Config: {info.vectors_config}")

    print("\nQdrantManager example usage finished.")
