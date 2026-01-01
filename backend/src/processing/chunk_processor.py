from typing import List
from src.processing.text_splitter import TextSplitter

class ChunkProcessor:
    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 200):
        """
        Initializes the ChunkProcessor with a TextSplitter instance.

        Args:
            chunk_size (int): The maximum number of tokens/characters per chunk.
            chunk_overlap (int): The number of tokens/characters to overlap between adjacent chunks.
        """
        self.text_splitter = TextSplitter(chunk_size=chunk_size, chunk_overlap=chunk_overlap)

    def process_text_into_chunks(self, text: str) -> List[str]:
        """
        Splits a single block of text into smaller, overlapping chunks.

        Args:
            text (str): The cleaned text to be chunked.

        Returns:
            List[str]: A list of text chunks.
        """
        if not text:
            return []
        return self.text_splitter.split_text(text)

    def process_documents_into_chunks(self, texts: List[str], metadatas: List[dict] = None) -> List[dict]:
        """
        Splits a list of texts (documents) into smaller, overlapping chunks,
        associating metadata with each chunk if provided.

        Args:
            texts (List[str]): A list of text strings, each representing a document.
            metadatas (List[dict], optional): A list of metadata dictionaries,
                                             one for each text in `texts`.

        Returns:
            List[dict]: A list of dictionaries, where each dictionary represents a chunk
                        with 'page_content' and 'metadata'.
        """
        if not texts:
            return []
        
        # The TextSplitter's create_documents method handles splitting and
        # associating metadata correctly.
        # It expands a single document into multiple chunks, each inheriting metadata.
        # If original texts have individual metadata, they will be passed through.
        return self.text_splitter.create_documents(texts, metadatas)


if __name__ == "__main__":
    # Example Usage
    long_document = (
        "This is the first paragraph of a very long document about Physical AI. "
        "It discusses the foundational concepts and early developments in the field. "
        "Physical AI aims to bridge the gap between artificial intelligence and the physical world. "
        "The second paragraph delves into the applications of Physical AI in robotics, "
        "especially in humanoid robots. These robots require sophisticated control systems "
        "and real-time decision-making capabilities to interact safely and efficiently with humans. "
        "Further sections might cover sensing, actuation, and learning algorithms for embodied systems. "
        "The third paragraph explores the challenges of deploying Physical AI systems in complex environments. "
        "These include robust perception, adaptable motor control, and ethical considerations. "
        "Future research directions involve soft robotics and advanced human-robot interaction. "
    ) * 3 # Make it even longer

    processor = ChunkProcessor(chunk_size=200, chunk_overlap=50)

    print("--- Processing single text into chunks ---")
    chunks_from_text = processor.process_text_into_chunks(long_document)
    for i, chunk in enumerate(chunks_from_text):
        print(f"Chunk {i+1} (Length: {len(chunk)}):\n'{chunk}'\n---")

    print("\n--- Processing texts with metadata into chunks ---")
    documents_to_process = [
        "Introduction to Physical AI: Bridging AI and the physical world.",
        "Applications in Humanoid Robotics: Control systems and human interaction."
    ]
    meta_info = [
        {"source_url": "http://example.com/intro", "chapter": "Introduction"},
        {"source_url": "http://example.com/robotics", "chapter": "Robotics"}
    ]

    chunks_from_docs = processor.process_documents_into_chunks(documents_to_process, meta_info)
    for i, doc_chunk in enumerate(chunks_from_docs):
        print(f"Chunk {i+1} (Length: {len(doc_chunk.page_content)}):\n'{doc_chunk.page_content}'")
        print(f"Metadata: {doc_chunk.metadata}\n---")
