from urllib.parse import urlparse
import re
from bs4 import BeautifulSoup
from typing import Dict, Optional

class MetadataExtractor:
    def __init__(self):
        """
        Initializes the MetadataExtractor.
        """
        pass

    def extract_metadata(self, url: str, html_content: Optional[str] = None) -> Dict[str, str]:
        """
        Extracts structured metadata from a URL and optionally its HTML content,
        tailored for Docusaurus sites.

        Args:
            url (str): The URL of the page.
            html_content (Optional[str]): The HTML content of the page (used for heading extraction).

        Returns:
            Dict[str, str]: A dictionary containing extracted metadata.
        """
        metadata = {"source_url": url}

        parsed_url = urlparse(url)
        path_segments = [s for s in parsed_url.path.split('/') if s]

        # Attempt to extract module, chapter, section from URL path
        if "docs" in path_segments:
            try:
                docs_index = path_segments.index("docs")
                content_path_segments = path_segments[docs_index + 1:]

                if content_path_segments:
                    # Heuristic for Docusaurus path structure:
                    # /docs/category/subcategory/doc-slug
                    # /docs/module1/ros2-basics
                    # /docs/intro
                    
                    if len(content_path_segments) >= 1:
                        # Attempt to get module from the first segment after 'docs/'
                        # e.g., 'module1' from /docs/module1/ros2-basics
                        module_candidate = content_path_segments[0].replace('-', ' ').title()
                        if re.match(r'Module\s+\d+', module_candidate):
                             metadata['module'] = module_candidate
                        else: # If it's not a numbered module, it might be a general category
                             metadata['module'] = content_path_segments[0].replace('-', ' ').title()

                    if len(content_path_segments) >= 2:
                        # Attempt to get chapter from the second segment
                        # e.g., 'ros2-basics' from /docs/module1/ros2-basics
                        metadata['chapter'] = content_path_segments[1].replace('-', ' ').title()

                    # The last segment is usually the document slug, which can also be a section/heading
                    if content_path_segments:
                        doc_slug = content_path_segments[-1]
                        metadata['section'] = doc_slug.replace('-', ' ').title()
                        metadata['heading'] = doc_slug.replace('-', ' ').title() # Default to section

            except ValueError:
                pass # 'docs' not found, or other path issues

        # Extract primary heading from HTML content if available
        if html_content:
            soup = BeautifulSoup(html_content, 'html.parser')
            # Look for the main title (h1 or similar)
            main_title = soup.find(['h1', 'h2'], class_=re.compile(r'header|title|heading'))
            if main_title:
                # Prioritize content within the main article/markdown area
                if 'section' not in metadata and 'heading' not in metadata:
                    metadata['section'] = main_title.get_text(strip=True)
                    metadata['heading'] = main_title.get_text(strip=True)
                else:
                    # If already extracted from URL, refine it with HTML heading if more descriptive
                    # or add it as a sub-heading
                    if 'section' in metadata and metadata['section'] == metadata['heading']:
                         html_heading_text = main_title.get_text(strip=True)
                         if len(html_heading_text) > len(metadata['heading']) + 5: # If HTML heading is much longer
                             metadata['heading'] = html_heading_text

        return metadata

if __name__ == "__main__":
    extractor = MetadataExtractor()

    # Example Docusaurus URLs
    urls = [
        "https://example.com/docs/intro",
        "https://example.com/docs/category/getting-started",
        "https://example.com/docs/module1/ros2-basics",
        "https://example.com/docs/module1/ros2-basics/setup-guide",
        "https://example.com/blog/2023/10/26/first-post",
        "https://example.com/some-other-page"
    ]

    print("--- Extracting Metadata from URLs ---")
    for url in urls:
        print(f"\nURL: {url}")
        metadata = extractor.extract_metadata(url)
        for key, value in metadata.items():
            print(f"  {key}: {value}")

    # Example with HTML content (simplified)
    html_example = """
    <html><body>
        <main>
            <h1 class="theme-doc-title">ROS2 Basics Setup Guide</h1>
            <p>Content goes here...</p>
        </main>
    </body></html>
    """
    print("\n--- Extracting Metadata from URL and HTML ---")
    url_with_html = "https://example.com/docs/module1/ros2-basics/setup-guide"
    metadata_html = extractor.extract_metadata(url_with_html, html_content=html_example)
    print(f"URL: {url_with_html}")
    for key, value in metadata_html.items():
        print(f"  {key}: {value}")
