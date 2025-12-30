import requests
from bs4 import BeautifulSoup
from qdrant_client import QdrantClient, models
from cohere import Client
from unstructured.cleaners.core import clean
import os
from dotenv import load_dotenv
import asyncio

def get_all_urls(base_url: str) -> list[str]:
    """
    Crawls the base URL to find all relevant book URLs.
    """
    try:
        response = requests.get(base_url)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, 'html.parser')

        # Find all links within the main content area that point to documentation pages
        # This might need to be adjusted based on the specific HTML structure of the website
        # Looking for links under a 'docs' path, typically within a navigation or main content div

        all_links = set()
        for link in soup.find_all('a', href=True):
            href = link['href']
            # Normalize href to handle relative URLs properly
            if href.startswith('/hackathon-humanoid-robotics-textbook/docs/'):
                full_url = f"https://Umm-e-Hani02.github.io{href}"
                all_links.add(full_url)
            elif href.startswith('/hackathon-humanoid-robotics-textbook/'):
                # Also capture direct links if they are within the book structure
                if not any(ext in href.lower() for ext in ['.png', '.jpg', '.jpeg', '.gif', '.svg', '.css', '.js', '.ico', '.pdf']):
                    full_url = f"https://Umm-e-Hani02.github.io{href}"
                    all_links.add(full_url)
            elif href.startswith('http') and 'hackathon-humanoid-robotics-textbook' in href:
                # Handle absolute URLs that contain the textbook path
                all_links.add(href)

        # Filter out any non-HTML links
        filtered_links = []
        for link in all_links:
            if not any(ext in link.lower() for ext in ['.png', '.jpg', '.jpeg', '.gif', '.svg', '.css', '.js', '.ico', '.pdf', '.zip', '.exe']):
                filtered_links.append(link)

        return filtered_links
    except requests.RequestException as e:
        print(f"Error fetching base URL {base_url}: {e}")
        return []

def extract_text_from_url(url: str) -> str:
    """
    Fetches the content of a given URL and extracts visible text.
    """
    try:
        response = requests.get(url)
        response.raise_for_status() # Raise an HTTPError for bad responses (4xx or 5xx)
        soup = BeautifulSoup(response.content, 'html.parser')

        # Remove script and style elements
        for script_or_style in soup(['script', 'style']):
            script_or_style.decompose()

        # Get text
        text = soup.get_text()

        # Clean extra whitespace
        cleaned_text = clean(text, extra_whitespace=True)
        return cleaned_text
    except requests.exceptions.RequestException as e:
        print(f"Error fetching URL {url}: {e}")
        return ""

def chunk_text(text: str, chunk_size: int = 1000, chunk_overlap: int = 200) -> list[str]:
    """
    Splits a long string of text into smaller, manageable chunks with overlap.
    """
    if not isinstance(text, str):
        return []

    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunks.append(text[start:end])
        start += chunk_size - chunk_overlap
        if start >= len(text):
            break
            
    return chunks

def embed(texts: list[str], co: Client) -> list[list[float]]:
    """
    Generates embeddings for a list of text strings using Cohere.
    """
    if not texts:
        return []
    response = co.embed(
        texts=texts,
        model="embed-english-v3.0",
        input_type="search_document"
    )
    return response.embeddings

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

def save_chunk_to_qdrant(qdrant_client: QdrantClient, collection_name: str, chunks: list[str], embeddings: list[list[float]], url: str):
    """
    Saves text chunks, their embeddings, and associated metadata to Qdrant.
    """
    points = []
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        # Generate a more reliable unique ID to avoid potential hash collisions
        unique_id = abs(hash(f"{url}-{i}-{len(chunk)}")) % (10**10)  # Limit ID size to prevent overflow
        points.append(
            models.PointStruct(
                id=unique_id, # Generate a unique ID for each chunk
                vector=embedding,
                payload={"text": chunk, "source_url": url, "chunk_id": i},
            )
        )

    if points:
        try:
            qdrant_client.upsert(
                collection_name=collection_name,
                points=points,
                wait=True,
            )
            print(f"Saved {len(points)} chunks from {url} to Qdrant.")
        except Exception as e:
            print(f"Error saving chunks from {url} to Qdrant: {e}")
            # If individual upsert fails, try one by one to identify problematic points
            for point in points:
                try:
                    qdrant_client.upsert(
                        collection_name=collection_name,
                        points=[point],
                        wait=True,
                    )
                except Exception as point_error:
                    print(f"Failed to save point with ID {point.id}: {point_error}")

async def main():
    load_dotenv()

    # Initialize Cohere Client
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    if COHERE_API_KEY is None:
        print("COHERE_API_KEY environment variable not set. Please set it to your Cohere API key.")
        exit(1)
    co = Client(api_key=COHERE_API_KEY)

    # Initialize Qdrant Client
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    if QDRANT_URL is None or QDRANT_API_KEY is None:
        print("QDRANT_URL or QDRANT_API_KEY environment variable not set. Please set them in your .env file.")
        exit(1)
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

    COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "physical-ai-book-rag")
    VECTOR_SIZE = 1024  # Cohere 'embed-english-v3.0' model size

    base_url = "https://Umm-e-Hani02.github.io/hackathon-humanoid-robotics-textbook/"
    
    # 1. Create Qdrant collection
    create_collection(qdrant_client, COLLECTION_NAME, VECTOR_SIZE)

    # 2. Get all URLs
    urls = get_all_urls(base_url)
    print(f"Found {len(urls)} URLs to process.")

    for url in urls:
        print(f"Processing URL: {url}")
        # 3. Extract text
        text = extract_text_from_url(url)
        if not text:
            continue

        # 4. Chunk text
        chunks = chunk_text(text)
        if not chunks:
            print(f"No chunks generated for {url}")
            continue

        # 5. Embed chunks
        embeddings = embed(chunks, co)
        if not embeddings:
            print(f"No embeddings generated for {url}")
            continue

        # 6. Save to Qdrant
        save_chunk_to_qdrant(qdrant_client, COLLECTION_NAME, chunks, embeddings, url)
    
    print("Book data processing initiated successfully.")

if __name__ == "__main__":
    # To run the ingestion pipeline directly
    print("Starting book data processing...")
    asyncio.run(main())
    print("Book data processing finished.")
