import pytest
from unittest.mock import patch, MagicMock
import requests # Added import
from src.crawler.docusaurus_crawler import DocusaurusCrawler

# Mock the requests.get to prevent actual network calls during testing
@pytest.fixture
def mock_requests_get():
    with patch('requests.get') as mock_get:
        yield mock_get

def test_normalize_base_url():
    crawler = DocusaurusCrawler("http://example.com/docs")
    assert crawler._normalize_base_url("http://example.com/docs") == "http://example.com/docs/"
    assert crawler._normalize_base_url("http://example.com/docs/") == "http://example.com/docs/"
    assert crawler._normalize_base_url("http://example.com") == "http://example.com/"

def test_is_internal_link():
    # Test internal links relative to the base domain, not specifically base_url path
    crawler = DocusaurusCrawler("http://example.com/") 
    assert crawler._is_internal_link("http://example.com/docs/page1") is True
    assert crawler._is_internal_link("http://example.com/page1") is True
    assert crawler._is_internal_link("/docs/page1") is True
    assert crawler._is_internal_link("https://external.com/page") is False
    assert crawler._is_internal_link("http://anotherexample.com/docs/page") is False

def test_get_absolute_url():
    crawler = DocusaurusCrawler("http://example.com/docs/")
    assert crawler._get_absolute_url("page1", "http://example.com/docs/") == "http://example.com/docs/page1"
    assert crawler._get_absolute_url("/page1", "http://example.com/docs/sub/") == "http://example.com/page1"
    assert crawler._get_absolute_url("http://example.com/full", "http://example.com/docs/") == "http://example.com/full"

def test_crawl_single_page_no_links(mock_requests_get):
    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.raise_for_status.return_value = None
    mock_response.text = "<html><body><p>Hello</p></body></html>"
    mock_requests_get.return_value = mock_response

    crawler = DocusaurusCrawler("http://example.com/")
    urls = crawler.crawl()

    assert len(urls) == 1
    assert "http://example.com/" in urls
    mock_requests_get.assert_called_once_with("http://example.com/", timeout=5)

def test_crawl_multiple_pages(mock_requests_get):
    mock_response_home = MagicMock()
    mock_response_home.status_code = 200
    mock_response_home.raise_for_status.return_value = None
    mock_response_home.text = """
        <html><body>
            <a href="/docs/intro">Intro</a>
            <a href="http://example.com/docs/module1">Module 1</a>
            <a href="http://external.com">External</a>
        </body></html>
    """

    mock_response_intro = MagicMock()
    mock_response_intro.status_code = 200
    mock_response_intro.raise_for_status.return_value = None
    mock_response_intro.text = "<html><body><a href='/docs/another'>Another Doc</a></body></html>"

    mock_response_module1 = MagicMock()
    mock_response_module1.status_code = 200
    mock_response_module1.raise_for_status.return_value = None
    mock_response_module1.text = "<html><body><a href='/docs/module1/subpage'>Subpage</a></body></html>"

    mock_response_another = MagicMock()
    mock_response_another.status_code = 200
    mock_response_another.raise_for_status.return_value = None
    mock_response_another.text = "<html><body><p>Final Page</p></body></html>"
    
    mock_response_subpage = MagicMock()
    mock_response_subpage.status_code = 200
    mock_response_subpage.raise_for_status.return_value = None
    mock_response_subpage.text = "<html><body><p>Subpage Content</p></body></html>"


    def side_effect(url, timeout):
        if url == "http://example.com/":
            return mock_response_home
        elif url == "http://example.com/docs/intro":
            return mock_response_intro
        elif url == "http://example.com/docs/module1":
            return mock_response_module1
        elif url == "http://example.com/docs/another":
            return mock_response_another
        elif url == "http://example.com/docs/module1/subpage":
            return mock_response_subpage
        raise ValueError(f"Unexpected URL: {url}")

    mock_requests_get.side_effect = side_effect

    crawler = DocusaurusCrawler("http://example.com/")
    urls = crawler.crawl()

    expected_urls = {
        "http://example.com/",
        "http://example.com/docs/intro",
        "http://example.com/docs/module1",
        "http://example.com/docs/another",
        "http://example.com/docs/module1/subpage",
    }

    assert len(urls) == len(expected_urls)
    assert set(urls) == expected_urls
    
    # Assert that requests.get was called for each expected URL exactly once
    assert len(mock_requests_get.call_args_list) == len(expected_urls)
    called_urls = [call[0][0] for call in mock_requests_get.call_args_list]
    assert set(called_urls) == expected_urls


def test_crawl_http_error(mock_requests_get):
    mock_response = MagicMock()
    mock_response.status_code = 404
    mock_response.raise_for_status.side_effect = requests.exceptions.HTTPError("404 Not Found")
    mock_response.text = "<html><body><a href='/docs/page'>Page</a></body></html>"
    
    mock_requests_get.return_value = mock_response

    crawler = DocusaurusCrawler("http://example.com/")
    urls = crawler.crawl()

    assert len(urls) == 1 # Only the base URL initially added
    assert "http://example.com/" in urls
    # Ensure requests.get was called for the base URL
    assert mock_requests_get.call_args_list[0].args[0] == "http://example.com/"


def test_crawl_request_exception(mock_requests_get):
    mock_requests_get.side_effect = requests.exceptions.ConnectionError("Connection Refused")

    crawler = DocusaurusCrawler("http://example.com/")
    urls = crawler.crawl()

    assert len(urls) == 1 # Only the base URL initially added
    assert "http://example.com/" in urls
    # Ensure requests.get was called for the base URL
    assert mock_requests_get.call_args_list[0].args[0] == "http://example.com/"

def test_crawl_duplicate_links(mock_requests_get):
    mock_response_home = MagicMock()
    mock_response_home.status_code = 200
    mock_response_home.raise_for_status.return_value = None
    mock_response_home.text = """
        <html><body>
            <a href="/docs/page1">Page 1</a>
            <a href="/docs/page1">Page 1 Duplicate</a>
            <a href="http://example.com/docs/page1">Page 1 Full URL</a>
            <a href="/static/image.png">Image (not a docusaurus doc)</a>
        </body></html>
    """
    mock_response_page1 = MagicMock()
    mock_response_page1.status_code = 200
    mock_response_page1.raise_for_status.return_value = None
    mock_response_page1.text = "<html><body><p>Content of Page 1</p></body></html>"

    def side_effect(url, timeout):
        if url == "http://example.com/":
            return mock_response_home
        elif url == "http://example.com/docs/page1":
            return mock_response_page1
        raise ValueError(f"Unexpected URL: {url}")
    
    mock_requests_get.side_effect = side_effect

    crawler = DocusaurusCrawler("http://example.com/")
    urls = crawler.crawl()

    expected_urls = {
        "http://example.com/", # base url is always included initially
        "http://example.com/docs/page1",
    }

    assert len(urls) == len(expected_urls)
    assert set(urls) == expected_urls
    
    # Assert that requests.get was called for each expected URL exactly once
    assert len(mock_requests_get.call_args_list) == len(expected_urls)
    called_urls = [call[0][0] for call in mock_requests_get.call_args_list]
    assert set(called_urls) == expected_urls


def test_crawl_only_docusaurus_paths(mock_requests_get):
    mock_response_home = MagicMock()
    mock_response_home.status_code = 200
    mock_response_home.raise_for_status.return_value = None
    mock_response_home.text = """
        <html><body>
            <a href="/docs/intro">Doc Intro</a>
            <a href="/blog/my-post">Blog Post</a>
            <a href="/api/endpoint">API Endpoint</a>
            <a href="/static/image.png">Image</a>
        </body></html>
    """
    mock_response_intro = MagicMock()
    mock_response_intro.status_code = 200
    mock_response_intro.raise_for_status.return_value = None
    mock_response_intro.text = "<html><body><p>Intro Content</p></body></html>"

    mock_response_blog = MagicMock()
    mock_response_blog.status_code = 200
    mock_response_blog.raise_for_status.return_value = None
    mock_response_blog.text = "<html><body><p>Blog Content</p></body></html>"

    def side_effect(url, timeout):
        if url == "http://example.com/":
            return mock_response_home
        elif url == "http://example.com/docs/intro":
            return mock_response_intro
        elif url == "http://example.com/blog/my-post":
            return mock_response_blog
        raise ValueError(f"Unexpected URL: {url}")

    mock_requests_get.side_effect = side_effect

    crawler = DocusaurusCrawler("http://example.com/")
    urls = crawler.crawl()

    expected_urls = {
        "http://example.com/", # base url is always included initially
        "http://example.com/docs/intro",
        "http://example.com/blog/my-post",
    }
    
    assert set(urls) == expected_urls
    # Assert that requests.get was called for each expected URL exactly once
    assert len(mock_requests_get.call_args_list) == len(expected_urls)
    called_urls = [call[0][0] for call in mock_requests_get.call_args_list]
    assert set(called_urls) == expected_urls