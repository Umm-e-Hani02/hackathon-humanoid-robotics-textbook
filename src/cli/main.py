import argparse
import requests
import os
import asyncio # Added this import
from src.utils.env_loader import load_env_variables, get_env_variable
from src.crawler.docusaurus_crawler import DocusaurusCrawler
from src.crawler.content_extractor import ContentExtractor
from src.processing.text_cleaner import TextCleaner
from src.processing.chunk_processor import ChunkProcessor
from src.processing.metadata_extractor import MetadataExtractor
from src.embedding.embedding_generator import EmbeddingGenerator
from src.storage.vector_store_manager import VectorStoreManager

def ingest_command(args):
    """
    Orchestrates the ingestion pipeline to crawl a Docusaurus site,
    process its content, generate embeddings, and store them in Qdrant.
    """
    load_env_variables()

    try:
        base_url = args.base_url or get_env_variable("DOCUSAURUS_BASE_URL")
        print(f"Starting ingestion for Docusaurus site: {base_url}")

        # 1. Crawl Docusaurus site
        print("Step 1: Crawling Docusaurus site...")
        crawler = DocusaurusCrawler(base_url)
        # Start crawling from the intro page to ensure we hit the docs content
        all_urls = asyncio.run(crawler.crawl(start_url=base_url + "docs/intro"))
        print(f"Found {len(all_urls)} unique URLs.")

        content_extractor = ContentExtractor()
        text_cleaner = TextCleaner()
        chunk_processor = ChunkProcessor()
        metadata_extractor = MetadataExtractor()
        embedding_generator = EmbeddingGenerator()
        vector_store_manager = VectorStoreManager()

        total_chunks_processed = 0
        all_embeddings = []
        all_payloads = []

        for url in all_urls:
            print(f"Processing URL: {url}")
            try:
                response = requests.get(url, timeout=10)
                response.raise_for_status()
                html_content = response.text
                
                # Debugging: Print raw HTML content
                print(f"Raw HTML content for {url}:\n{html_content[:500]}...\n") # Print first 500 chars

                # 2. Extract content
                extracted_text = content_extractor.extract_text(html_content)
                
                # Debugging: Print extracted text
                print(f"Extracted text for {url}:\n{extracted_text[:500]}...\n") # Print first 500 chars
                
                if not extracted_text:
                    print(f"  No significant text extracted from {url}. Skipping.")
                    continue

                # 3. Clean text
                cleaned_text = text_cleaner.clean_text(extracted_text)
                if not cleaned_text:
                    print(f"  Text became empty after cleaning for {url}. Skipping.")
                    continue

                # 4. Generate chunks and metadata
                # First, extract basic metadata from URL and HTML
                base_metadata = metadata_extractor.extract_metadata(url, html_content)
                
                # Split the cleaned text into chunks
                chunks_with_metadata = chunk_processor.process_documents_into_chunks(
                    texts=[cleaned_text],
                    metadatas=[base_metadata]
                )

                chunk_texts = [doc.page_content for doc in chunks_with_metadata]
                chunk_payloads = [doc.metadata for doc in chunks_with_metadata]

                # 5. Generate embeddings for chunks
                if chunk_texts:
                    embeddings = embedding_generator.generate_embeddings(chunk_texts)
                    
                    # Add text_content to payload for storage, as per qdrant_schema.md
                    for i, payload in enumerate(chunk_payloads):
                        payload['text_content'] = chunk_texts[i]
                        # Ensure the ID for upsert is stable - using a hash of URL + chunk content
                        # This allows for update-safe ingestion (T025) later.
                        # For now, just generate one. A UUID or hash is good.
                        payload['id'] = str(hash(url + chunk_texts[i])) # Simple stable ID for chunks

                    all_embeddings.extend(embeddings)
                    all_payloads.extend(chunk_payloads)
                    total_chunks_processed += len(chunk_texts)
                    print(f"  Generated {len(chunk_texts)} chunks and embeddings.")

            except requests.exceptions.RequestException as e:
                print(f"  Error fetching {url}: {e}")
            except Exception as e:
                print(f"  Error processing {url}: {e}")
        
        # 6. Store vectors and metadata in Qdrant
        print(f"\nStep 6: Storing {len(all_embeddings)} vectors in Qdrant...")
        if all_embeddings:
            # Extract IDs from payloads for upsert
            ids_for_upsert = [p['id'] for p in all_payloads]
            vector_store_manager.upsert_vectors_batch(all_embeddings, all_payloads, ids=ids_for_upsert)
            print(f"Successfully processed and stored {total_chunks_processed} chunks.")
        else:
            print("No embeddings to store.")

    except ValueError as e:
        print(f"Configuration error: {e}")
        print("Please ensure DOCUSAURUS_BASE_URL, COHERE_API_KEY, QDRANT_API_KEY, and QDRANT_CLUSTER_URL are set in your .env file.")
    except Exception as e:
        print(f"An unexpected error occurred during ingestion: {e}")

def main():
    parser = argparse.ArgumentParser(description="RAG Ingestion Pipeline CLI")
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Ingest command
    ingest_parser = subparsers.add_parser("ingest", help="Ingest content from a Docusaurus site into Qdrant")
    ingest_parser.add_argument("--base-url", type=str, help="Base URL of the Docusaurus site to crawl (overrides .env)")
    ingest_parser.set_argument_defaults(func=ingest_command)

    args = parser.parse_args()

    if args.command:
        args.func(args)
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
