import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from typing import Set, List
from src.utils.env_loader import get_env_variable, load_env_variables

class DocusaurusCrawler:
    def __init__(self, base_url: str):
        self.base_url = self._normalize_base_url(base_url)
        self.visited_urls: Set[str] = set()
        self.internal_urls: Set[str] = set()
        print(f"Crawler initialized for base URL: {self.base_url}")

    def _normalize_base_url(self, url: str) -> str:
        """Ensures the base URL ends with a slash and is properly formatted."""
        if not url.endswith('/'):
            url += '/'
        return url

    def _is_internal_link(self, url: str) -> bool:
        """Checks if a URL is an internal link relative to the base URL."""
        return url.startswith(self.base_url) or url.startswith('/')

    def _get_absolute_url(self, href: str, current_url: str) -> str:
        """Converts a relative URL to an absolute URL."""
        return urljoin(current_url, href)

    def crawl(self, start_url: str = None) -> List[str]:
        """
        Crawls the Docusaurus website starting from the base URL or a specified start_url.

        Args:
            start_url (str, optional): The URL to start crawling from. If None, uses the base_url.

        Returns:
            List[str]: A list of all unique internal Docusaurus document URLs found.
        """
        if start_url is None:
            start_url = self.base_url
        else:
            start_url = self._normalize_base_url(start_url)
            if not self._is_internal_link(start_url):
                raise ValueError(f"Start URL {start_url} is not an internal link to {self.base_url}")

        queue = [start_url]
        self.visited_urls.add(start_url)
        self.internal_urls.add(start_url)

        while queue:
            current_url = queue.pop(0)
            print(f"Crawling: {current_url}")

            try:
                response = requests.get(current_url, timeout=5)
                response.raise_for_status() # Raise an exception for HTTP errors
            except requests.exceptions.RequestException as e:
                print(f"Error crawling {current_url}: {e}")
                continue

            soup = BeautifulSoup(response.text, 'html.parser')

            # Find all <a> tags with href attributes
            for link in soup.find_all('a', href=True):
                href = link['href']
                absolute_link = self._get_absolute_url(href, current_url)
                
                # Normalize the link by removing fragments and query parameters for comparison
                parsed_link = urlparse(absolute_link)
                normalized_link = urljoin(parsed_link.scheme + '://' + parsed_link.netloc, parsed_link.path)
                
                # Check if it's an internal Docusaurus link and not visited yet
                if self._is_internal_link(normalized_link) and normalized_link not in self.visited_urls:
                    if normalized_link.startswith(self.base_url + 'docs/') or \
                       normalized_link.startswith(self.base_url + 'blog/'):
                        self.visited_urls.add(normalized_link)
                        self.internal_urls.add(normalized_link)
                        queue.append(normalized_link)
                        print(f"  Found new internal link: {normalized_link}")
        
        return list(self.internal_urls)

if __name__ == "__main__":
    # Example usage:
    # Set DOCUSAURUS_BASE_URL in your .env file or directly here for testing
    # For local testing, you might need a local Docusaurus server running
    load_env_variables()
    
    try:
        base_docusaurus_url = get_env_variable("DOCUSAURUS_BASE_URL", "http://localhost:3000/") # Default for local testing
        print(f"Attempting to crawl: {base_docusaurus_url}")
        crawler = DocusaurusCrawler(base_docusaurus_url)
        found_urls = crawler.crawl()

        print("\n--- Crawling Complete ---")
        print(f"Found {len(found_urls)} unique internal URLs:")
        for url in sorted(found_urls):
            print(url)
    except ValueError as e:
        print(f"Configuration error: {e}. Please ensure DOCUSAURUS_BASE_URL is set.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
