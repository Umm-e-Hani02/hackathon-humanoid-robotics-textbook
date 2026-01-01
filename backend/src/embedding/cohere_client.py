import cohere
from src.utils.env_loader import get_env_variable, load_env_variables

class CohereClientManager:
    def __init__(self):
        load_env_variables()  # Ensure environment variables are loaded
        self.cohere_api_key = get_env_variable("COHERE_API_KEY")
        self.client = cohere.Client(self.cohere_api_key)

    def get_client(self) -> cohere.Client:
        """
        Returns the initialized Cohere client.
        """
        return self.client

    def get_embedding_model(self, model_name: str = "embed-english-v3.0"):
        """
        Retrieves a Cohere embedding model.

        Args:
            model_name (str): The name of the embedding model to use.

        Returns:
            str: The name of the model. (In Cohere SDK, the model is specified directly in the embed call)
        """
        # Cohere client doesn't explicitly 'get' a model like this,
        # you specify it in the embed method directly.
        # This method is more of a placeholder to indicate which model will be used.
        print(f"Using Cohere embedding model: {model_name}")
        return model_name

if __name__ == "__main__":
    # Example Usage:
    # Ensure COHERE_API_KEY is set in your .env file
    print("Initializing CohereClientManager...")
    manager = CohereClientManager()
    
    cohere_client = manager.get_client()
    print(f"Cohere client initialized: {cohere_client is not None}")

    model = manager.get_embedding_model()
    print(f"Configured embedding model: {model}")

    # Example of calling embed (requires actual API key and text)
    # try:
    #     response = cohere_client.embed(
    #         texts=["hello world", "goodbye world"],
    #         model=model,
    #         input_type="classification" # or "search_query", "search_document"
    #     )
    #     print("Embeddings generated successfully (example):")
    #     print(response.embeddings[:1])
    # except cohere.CohereError as e:
    #     print(f"Error embedding text: {e}")
