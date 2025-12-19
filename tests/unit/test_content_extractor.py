import pytest
from src.crawler.content_extractor import ContentExtractor

@pytest.fixture
def extractor():
    return ContentExtractor()

def test_extract_text_simple_html(extractor):
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
    expected_text = "Main Title This is the main content paragraph. Another paragraph in main."
    assert extractor.extract_text(html_simple) == expected_text

def test_extract_text_docusaurus_like_html(extractor):
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
    expected_text = "Docusaurus Doc Title This is the actual content of a Docusaurus documentation page. It can have multiple paragraphs and rich formatting."
    assert extractor.extract_text(html_docusaurus) == expected_text

def test_extract_text_no_body(extractor):
    html_no_body = "<html><head><title>No Body</title></head></html>"
    assert extractor.extract_text(html_no_body) is None

def test_extract_text_empty_html(extractor):
    assert extractor.extract_text("") is None

def test_extract_text_only_boilerplate(extractor):
    html_only_boilerplate = """
    <html><body>
        <header>Only Header</header>
        <nav>Only Nav</nav>
        <script>console.log("hi");</script>
        <footer>Only Footer</footer>
    </body></html>
    """
    assert extractor.extract_text(html_only_boilerplate) == ""

def test_extract_text_multiple_main_content_candidates(extractor):
    html = """
    <html><body>
        <div class="main-wrapper">Content from main wrapper</div>
        <div class="markdown">Content from markdown</div>
        <article>Content from article</article>
    </body></html>
    """
    expected_text = "Content from main wrapper Content from markdown Content from article"
    assert extractor.extract_text(html) == expected_text

def test_extract_text_with_comments_and_scripts(extractor):
    html = """
    <html><body>
        <!-- This is a comment -->
        <p>Main text.</p>
        <script>var x = 1;</script>
        <p>More text.</p>
    </body></html>
    """
    expected_text = "Main text. More text."
    assert extractor.extract_text(html) == expected_text

def test_extract_text_docusaurus_specific_classes(extractor):
    html = """
    <html><body>
        <div class="navbar">Nav stuff</div>
        <div class="sidebar">Sidebar stuff</div>
        <main>
            <div class="docItemContainer_o7d9">Important Document Content</div>
        </main>
        <div class="theme-doc-footer-edit">Edit link</div>
    </body></html>
    """
    expected_text = "Important Document Content"
    assert extractor.extract_text(html) == expected_text

def test_extract_text_with_h1_and_p_tags(extractor):
    html = """
    <html><body>
        <h1>Title One</h1>
        <p>Paragraph one content.</p>
        <h2>Subtitle Two</h2>
        <p>Paragraph two content.</p>
    </body></html>
    """
    expected_text = "Title One Paragraph one content. Subtitle Two Paragraph two content."
    # The current extractor design keeps all body text if specific main containers are not found,
    # or if they are, it aggregates them. The boilerplate removal targets specific CSS classes.
    # So, general h1/p tags without boilerplate class should remain.
    assert extractor.extract_text(html) == expected_text

