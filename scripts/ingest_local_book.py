import os
import glob
from qdrant_client import QdrantClient, models
from cohere import Client
from unstructured.cleaners.core import clean
from dotenv import load_dotenv
import asyncio
import hashlib

def get_local_book_files(base_path: str) -> list[str]:
    """
    Gets all markdown files from the local book directory.
    """
    md_files = glob.glob(os.path.join(base_path, "**", "*.md"), recursive=True)
    return md_files

def extract_text_from_md_file(file_path: str) -> str:
    """
    Extracts text content from a markdown file.
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

            # Extract just the main content, removing frontmatter if present
            lines = content.split('\n')
            if lines and lines[0].startswith('---'):  # Skip frontmatter
                try:
                    # Find the end of frontmatter
                    end_frontmatter_idx = -1
                    for i, line in enumerate(lines[1:], 1):
                        if line.strip() == '---':
                            end_frontmatter_idx = i
                            break
                    if end_frontmatter_idx != -1:
                        content = '\n'.join(lines[end_frontmatter_idx + 1:])
                except:
                    pass  # If frontmatter parsing fails, use full content

            return content
    except Exception as e:
        print(f"Error reading file {file_path}: {e}")
        return ""

def chunk_text(text: str, chunk_size: int = 1000, chunk_overlap: int = 200) -> list[str]:
    """
    Splits a long string of text into smaller, manageable chunks with overlap.
    """
    if not isinstance(text, str) or not text.strip():
        return []

    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunks.append(text[start:end])
        start += chunk_size - chunk_overlap
        if start >= len(text):
            break

    # Filter out empty or whitespace-only chunks
    return [chunk for chunk in chunks if chunk.strip()]

def embed(texts: list[str], co: Client) -> list[list[float]]:
    """
    Generates embeddings for a list of text strings using Cohere.
    """
    if not texts:
        return []

    # Cohere has a limit on the number of texts per request, so we need to batch
    batch_size = 96  # Conservative batch size to avoid rate limits
    all_embeddings = []

    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]
        try:
            response = co.embed(
                texts=batch,
                model="embed-english-v3.0",
                input_type="search_document"
            )
            all_embeddings.extend(response.embeddings)
        except Exception as e:
            print(f"Error embedding batch {i//batch_size + 1}: {e}")
            # Return empty embeddings for failed batch to maintain alignment
            all_embeddings.extend([[]] * len(batch))

    # Filter out any empty embeddings
    return [emb for emb in all_embeddings if emb]

def create_collection(qdrant_client: QdrantClient, collection_name: str, vector_size: int):
    """
    Creates a new collection in Qdrant if it doesn't already exist.
    """
    if not qdrant_client.collection_exists(collection_name=collection_name):
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
        )
        print(f"Collection '{collection_name}' created with vector size {vector_size} and COSINE distance.")
    else:
        print(f"Collection '{collection_name}' already exists.")

def save_chunk_to_qdrant(qdrant_client: QdrantClient, collection_name: str, chunks: list[str], embeddings: list[list[float]], source_file: str):
    """
    Saves text chunks, their embeddings, and associated metadata to Qdrant.
    """
    points = []
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        if not embedding:  # Skip if embedding failed
            continue

        # Generate a unique ID based on file path and chunk index to avoid collisions
        unique_id_str = f"{source_file}-{i}-{len(chunk)}"
        unique_id = abs(int(hashlib.md5(unique_id_str.encode()).hexdigest()[:10], 16)) % (10**10)  # Limit to 10 digits

        points.append(
            models.PointStruct(
                id=unique_id,  # Generate a unique ID for each chunk
                vector=embedding,
                payload={"text": chunk.strip(), "source_file": source_file, "chunk_id": i},
            )
        )

    if points:
        try:
            qdrant_client.upsert(
                collection_name=collection_name,
                points=points,
                wait=True,
            )
            print(f"Saved {len(points)} chunks from {source_file} to Qdrant.")
        except Exception as e:
            print(f"Error saving chunks from {source_file} to Qdrant: {e}")
            # If bulk upsert fails, try one by one to identify problematic points
            successful = 0
            for point in points:
                try:
                    qdrant_client.upsert(
                        collection_name=collection_name,
                        points=[point],
                        wait=True,
                    )
                    successful += 1
                except Exception as point_error:
                    print(f"Failed to save point with ID {point.id}: {point_error}")
            print(f"Successfully saved {successful} out of {len(points)} chunks from {source_file}")

async def main():
    load_dotenv()

    # Initialize Cohere Client
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    if COHERE_API_KEY is None:
        print("COHERE_API_KEY environment variable not set. Please set it to your Cohere API key.")
        exit(1)
    co = Client(api_key=COHERE_API_KEY)

    # Initialize Qdrant Client - use Qdrant Cloud
    # Strip whitespace from environment variables to fix the URL issue
    QDRANT_URL = os.getenv("QDRANT_URL", "").strip()
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "").strip()
    if not QDRANT_URL or not QDRANT_API_KEY:
        print("QDRANT_URL or QDRANT_API_KEY environment variable not set or contains only whitespace. Please set them in your .env file.")
        exit(1)

    try:
        qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY, timeout=30)
    except Exception as e:
        print(f"Failed to connect to Qdrant Cloud: {e}")
        print("Please ensure your Qdrant Cloud instance is accessible and credentials are correct.")
        exit(1)

    COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "physical-ai-book-rag")
    VECTOR_SIZE = 1024  # Cohere 'embed-english-v3.0' model size

    # Use the local book content directory
    base_path = "physical-ai-and-humanoid-robots/docs"
    print(f"Processing local book content from: {base_path}")

    # 1. Check if collection exists
    if not qdrant_client.collection_exists(collection_name=COLLECTION_NAME):
        create_collection(qdrant_client, COLLECTION_NAME, VECTOR_SIZE)
    else:
        print(f"Collection '{COLLECTION_NAME}' already exists, will append to existing data.")

    # 2. Get all markdown files
    md_files = get_local_book_files(base_path)
    print(f"Found {len(md_files)} markdown files to process.")

    total_chunks = 0
    for file_path in md_files:
        print(f"Processing file: {file_path}")

        # 3. Extract text
        text = extract_text_from_md_file(file_path)
        if not text.strip():
            print(f"  No content extracted from {file_path}")
            continue

        # 4. Chunk text
        chunks = chunk_text(text)
        if not chunks:
            print(f"  No chunks generated for {file_path}")
            continue

        print(f"  Generated {len(chunks)} chunks")

        # 5. Embed chunks
        embeddings = embed(chunks, co)
        if not embeddings or len(embeddings) != len(chunks):
            print(f"  Embeddings mismatch for {file_path} (chunks: {len(chunks)}, embeddings: {len(embeddings)})")
            continue

        # 6. Save to Qdrant
        save_chunk_to_qdrant(qdrant_client, COLLECTION_NAME, chunks, embeddings, file_path)
        total_chunks += len(chunks)

    print(f"Book data processing completed. Total new chunks saved: {total_chunks}")

    # Get final count
    try:
        collection_info = qdrant_client.get_collection(collection_name=COLLECTION_NAME)
        print(f"Total points in collection after ingestion: {collection_info.points_count}")
    except Exception as e:
        print(f"Could not retrieve final collection count: {e}")

if __name__ == "__main__":
    # To run the ingestion pipeline directly
    print("Starting local book data processing...")
    asyncio.run(main())
    print("Book data processing finished.")