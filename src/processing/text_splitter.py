from langchain_text_splitters import RecursiveCharacterTextSplitter
from typing import List

class TextSplitter:
    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 200):
        """
        Initializes the TextSplitter with a token-aware RecursiveCharacterTextSplitter.

        Args:
            chunk_size (int): The maximum number of tokens per chunk.
            chunk_overlap (int): The number of tokens to overlap between adjacent chunks.
        """
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=self.chunk_size,
            chunk_overlap=self.chunk_overlap,
            length_function=len,  # This will count characters, not tokens. For true token awareness
                                  # a specific tokenizer needs to be passed.
                                  # For simplicity, character count is used here as a proxy.
                                  # For a more precise token count, integrate a tokenizer here.
            add_start_index=True,
        )

    def split_text(self, text: str) -> List[str]:
        """
        Splits a given text into smaller, token-aware chunks.

        Args:
            text (str): The input text to be split.

        Returns:
            List[str]: A list of text chunks.
        """
        return self.text_splitter.split_text(text)

    def create_documents(self, texts: List[str], metadatas: List[dict] = None) -> List[dict]:
        """
        Creates 'documents' (dictionaries with 'page_content' and 'metadata')
        from texts and optional metadatas.

        Args:
            texts (List[str]): A list of texts to be split and converted to documents.
            metadatas (List[dict], optional): A list of metadata dictionaries, one for each text.

        Returns:
            List[dict]: A list of dictionaries, each representing a document.
        """
        if metadatas:
            return self.text_splitter.create_documents(texts, metadatas)
        return self.text_splitter.create_documents(texts)


if __name__ == "__main__":
    # Example Usage
    long_text = (
        "This is a very long document that needs to be split into smaller chunks. "
        "Token-aware splitting ensures that we don't cut in the middle of important words "
        "or concepts, making the chunks more meaningful for downstream tasks like embedding "
        "and retrieval. Overlapping chunks also help to maintain context across boundaries. "
        "This example text will demonstrate how the RecursiveCharacterTextSplitter "
        "handles different lengths and overlaps. We will set a chunk size of 100 "
        "and an overlap of 20 characters to see the effect." * 5
    )

    print("Initializing TextSplitter with chunk_size=100, chunk_overlap=20...")
    splitter = TextSplitter(chunk_size=100, chunk_overlap=20)

    print("\nSplitting text into chunks...")
    chunks = splitter.split_text(long_text)

    for i, chunk in enumerate(chunks):
        print(f"\n--- Chunk {i+1} (Length: {len(chunk)}) ---")
        print(chunk)
        if hasattr(chunk, 'metadata') and chunk.metadata:
            print(f"Metadata: {chunk.metadata}")

    # Example with metadatas
    print("\nSplitting text and adding metadata...")
    texts_with_metadata = ["First part of a document.", "Second part of the same document."]
    metadatas_list = [{"source": "doc1", "part": 1}, {"source": "doc1", "part": 2}]
    
    docs = splitter.create_documents(texts_with_metadata, metadatas_list)
    for i, doc in enumerate(docs):
        print(f"\n--- Document {i+1} ---")
        print(f"Content: {doc.page_content}")
        print(f"Metadata: {doc.metadata}")
