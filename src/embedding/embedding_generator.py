from typing import List
import cohere
from src.embedding.cohere_client import CohereClientManager

class EmbeddingGenerator:
    def __init__(self, model_name: str = "embed-english-v3.0"):
        """
        Initializes the EmbeddingGenerator with a CohereClientManager.

        Args:
            model_name (str): The name of the Cohere embedding model to use.
        """
        self.cohere_manager = CohereClientManager()
        self.cohere_client = self.cohere_manager.get_client()
        self.model_name = model_name

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generates embeddings for a list of text chunks using the Cohere client.

        Args:
            texts (List[str]): A list of text chunks.

        Returns:
            List[List[float]]: A list of embeddings, where each embedding is a list of floats.
        
        Raises:
            cohere.CohereError: If there's an issue with the Cohere API call.
        """
        if not texts:
            return []

        try:
            response = self.cohere_client.embed(
                texts=texts,
                model=self.model_name,
                input_type="search_document" # or "classification", "search_query", "clustering"
            )
            return response.embeddings
        except cohere.CohereError as e:
            print(f"Error generating embeddings with Cohere: {e}")
            raise # Re-raise the exception after logging

if __name__ == "__main__":
    # Example Usage
    # Ensure COHERE_API_KEY is set in your .env file
    try:
        generator = EmbeddingGenerator()
        
        sample_texts = [
            "This is the first sentence.",
            "This is the second sentence, talking about something different.",
            "A third piece of text to embed.",
            "The quick brown fox jumps over the lazy dog."
        ]
        print(f"Generating embeddings for {len(sample_texts)} texts...")
        embeddings = generator.generate_embeddings(sample_texts)
        
        if embeddings:
            print(f"Generated {len(embeddings)} embeddings.")
            print(f"Dimension of first embedding: {len(embeddings[0])}")
            print(f"First embedding (first 5 values): {embeddings[0][:5]}...")
        else:
            print("No embeddings were generated.")

    except ValueError as e:
        print(f"Configuration error: {e}. Please ensure COHERE_API_KEY is set.")
    except cohere.CohereError as e:
        print(f"Cohere API error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
