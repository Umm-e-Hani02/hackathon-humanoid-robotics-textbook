from bs4 import BeautifulSoup, Comment
from typing import Optional

class ContentExtractor:
    def __init__(self):
        """
        Initializes the ContentExtractor.
        """
        pass

    def extract_text(self, html_content: str) -> Optional[str]:
        """
        Extracts the main textual content from an HTML page, excluding boilerplate.

        Args:
            html_content (str): The full HTML content of a page.

        Returns:
            Optional[str]: The extracted main text content, or None if no significant content is found.
        """
        if not html_content:
            return None

        soup = BeautifulSoup(html_content, 'html.parser')

        print(f"--- Before boilerplate removal ---")
        print(soup.prettify()[:1000]) # Print first 1000 chars for brevity

        # Remove script and style elements
        for script_or_style in soup(['script', 'style', 'noscript', 'link']):
            script_or_style.decompose()

        # Remove comments
        for comment in soup.find_all(string=lambda text: isinstance(text, Comment)):
            comment.extract()

        # Identify and remove common boilerplate elements in Docusaurus and general web pages
        boilerplate_selectors = [
            'header',           # Page header
            'footer',           # Page footer
            'nav',              # Navigation bars
            '.navbar',          # Docusaurus navbar
            '.sidebar',         # Docusaurus sidebar
            '.table-of-contents', # Docusaurus table of contents
            '.theme-doc-footer-edit', # Docusaurus edit link
            '.pagination-nav',  # Docusaurus pagination
            '.doc-rating',      # Docusaurus rating component
            '.hash-link',       # Docusaurus heading anchor links
            '.theme-doc-breadcrumbs', # Docusaurus breadcrumbs
            '#docusaurus_tag_html_inject', # Docusaurus specific injects
            '#__docusaurus_skip_to_content_id', # Docusaurus skip link
            '.menu',            # Generic menu elements
            '.ad',              # Advertisements
            '.related-posts',   # Related articles/posts
            '.footer',          # Generic footer (if not already covered by 'footer')
            '.skipToContent_LXw1', # Docusaurus skip to content button
            '.docItemFooter_shcR', # Docusaurus doc item footer
            '.theme-last-updated', # Docusaurus last updated info
            '.alert',           # Docusaurus alert components (might be fine to keep, but often informational not core)
            '.tabs-container',  # Docusaurus tabs (often interactive, not pure content)
            '.button',          # Generic buttons (not usually core text content)
            '.hero',            # Often marketing banner, sometimes contains content. Remove if it's purely decorative.
        ]

        for selector in boilerplate_selectors:
            for element in soup.select(selector):
                element.decompose()

        print(f"--- After boilerplate removal ---")
        print(soup.prettify()[:1000]) # Print first 1000 chars for brevity

        # Prioritize main content areas for extraction
        # Docusaurus content is often inside <main>, <article>, or specific `div`s
        # We look for the most specific content containers first.
        main_content_elements = []
        
        # Look for docusaurus specific content containers
        docusaurus_content_divs = soup.find_all('div', class_=['markdown', 'docItemContainer_o7d9'])
        if docusaurus_content_divs:
            main_content_elements.extend(docusaurus_content_divs)
        
        # Fallback to general main content tags if specific divs are not found or are empty
        if not main_content_elements or all(not el.get_text(strip=True) for el in main_content_elements):
            general_main_tags = soup.find_all(['main', 'article'])
            main_content_elements.extend(general_main_tags)

        if main_content_elements:
            print(f"--- Found main content elements ---")
            for i, element in enumerate(main_content_elements):
                print(f"Element {i+1} tag: {element.name}, classes: {element.get('class')}")
                print(f"Element {i+1} text (first 200 chars): {element.get_text(separator=' ', strip=True)[:200]}...")
        else:
            print(f"--- No specific main content elements found, falling back to body ---")

        # If specific main content elements are found, combine their text
        if main_content_elements:
            text_parts = []
            for element in main_content_elements:
                text_parts.append(element.get_text(separator=' ', strip=True))
            return ' '.join(text_parts).strip()
        
        # Fallback: if no specific main content elements, extract text from the body
        # This might include more boilerplate but ensures some content is returned.
        return soup.body.get_text(separator=' ', strip=True) if soup.body else None

if __name__ == "__main__":
    extractor = ContentExtractor()

    # Example 1: Simple HTML
    html_simple = """
    <html>
    <head><title>Test Page</title></head>
    <body>
        <header>Header content</header>
        <main>
            <h1>Main Title</h1>
            <p>This is the main content paragraph.</p>
            <nav>Navigation links</nav>
            <p>Another paragraph in main.</p>
        </main>
        <footer>Footer text</footer>
        <script>alert('hello');</script>
    </body>
    </html>
    """
    print("--- Example 1: Simple HTML ---")
    extracted_text = extractor.extract_text(html_simple)
    print(extracted_text)
    # Expected: "Main Title This is the main content paragraph. Another paragraph in main."

    # Example 2: Docusaurus-like HTML snippet (simplified)
    html_docusaurus = """
    <html lang="en">
    <body>
        <div id="__docusaurus">
            <nav class="navbar"></nav>
            <div class="main-wrapper mainWrapper_e7z6">
                <aside class="sidebar"></aside>
                <main class="docMainContainer_gTbr">
                    <div class="container padding-top--md padding-bottom--lg">
                        <div class="row">
                            <div class="col col--3">
                                <div class="table-of-contents"></div>
                            </div>
                            <div class="col col--9">
                                <article>
                                    <nav class="theme-doc-breadcrumbs" aria-label="Breadcrumbs"></nav>
                                    <div class="markdown">
                                        <h1>Docusaurus Doc Title <a class="hash-link" href="#docusaurus-doc-title" title="Direct link to Docusaurus Doc Title">#</a></h1>
                                        <p>This is the actual content of a Docusaurus documentation page.</p>
                                        <p>It can have multiple paragraphs and <strong>rich formatting</strong>.</p>
                                        <div class="theme-doc-footer-edit"></div>
                                    </div>
                                    <footer class="docItemFooter_shcR"></footer>
                                </article>
                            </div>
                        </div>
                    </div>
                </main>
            </div>
            <footer class="footer"></footer>
            <script src="/js/some_script.js"></script>
        </div>
    </body>
    </html>
    """
    print("\n--- Example 2: Docusaurus-like HTML ---")
    extracted_text_docusaurus = extractor.extract_text(html_docusaurus)
    print(extracted_text_docusaurus)
    # Expected: "Docusaurus Doc Title This is the actual content of a Docusaurus documentation page. It can have multiple paragraphs and rich formatting."

    # Example 3: HTML with no body
    html_no_body = "<html><head><title>No Body</title></head></html>"
    print("\n--- Example 3: HTML with no body ---")
    extracted_text_no_body = extractor.extract_text(html_no_body)
    print(extracted_text_no_body)
    # Expected: None
