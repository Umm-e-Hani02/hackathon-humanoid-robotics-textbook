
import asyncio
from typing import Set, List
from src.crawler.docusaurus_crawler import DocusaurusCrawler
from src.storage.qdrant_client import QdrantManager

def get_crawled_urls() -> Set[str]:
    """Crawls the website to get all unique internal URLs."""
    print("----------- CRAWLING WEBSITE ----------- ")
    try:
        crawler = DocusaurusCrawler(base_url="https://www.physical-ai.com/")
        urls = crawler.crawl()
        print(f"Found {len(urls)} URLs.")
        return set(urls)
    except Exception as e:
        print(f"Error crawling website: {e}")
        return set()

def get_indexed_urls() -> Set[str]:
    """Retrieves all unique source_url values from the Qdrant index."""
    print("\n----------- QUERYING VECTOR DB ----------- ")
    indexed_urls = set()
    try:
        qdrant_manager = QdrantManager()
        collection_name = qdrant_manager.collection_name
        print(f"Querying collection: {collection_name}")

        response = qdrant_manager.client.scroll(
            collection_name=collection_name,
            limit=1000,
            with_payload=["source_url"],
            with_vectors=False
        )
        
        points = response[0]
        next_page_offset = response[1]

        while points:
            for point in points:
                source_url = point.payload.get("source_url")
                if source_url:
                    indexed_urls.add(source_url)
            
            if next_page_offset is None:
                break
                
            response = qdrant_manager.client.scroll(
                collection_name=collection_name,
                limit=1000,
                with_payload=["source_url"],
                with_vectors=False,
                offset=next_page_offset
            )
            points = response[0]
            next_page_offset = response[1]


        print(f"Found {len(indexed_urls)} indexed URLs.")
        return indexed_urls
    except Exception as e:
        print(f"Error querying Qdrant: {e}")
        return set()

def compare_urls(crawled_urls: Set[str], indexed_urls: Set[str]):
    """Compares the two sets of URLs and prints the differences."""
    print("\n----------- COMPARING DATA ----------- ")

    missing_from_index = crawled_urls - indexed_urls
    added_to_index_but_not_crawled = indexed_urls - crawled_urls

    if not missing_from_index and not added_to_index_but_not_crawled:
        print("✅ Success: All deployed book pages have been successfully embedded and indexed.")
        return

    if missing_from_index:
        print("\n❌ Error: The following URLs were found on the website but are MISSING from the vector database:")
        for url in sorted(missing_from_index):
            print(f"  - {url}")
    
    if added_to_index_but_not_crawled:
        print("\n⚠️ Warning: The following URLs are in the vector database but were NOT FOUND on the live website (they may be old or obsolete):")
        for url in sorted(added_to_index_but_not_crawled):
            print(f"  - {url}")

if __name__ == "__main__":
    crawled = get_crawled_urls()
    indexed = get_indexed_urls()
    compare_urls(crawled, indexed)
