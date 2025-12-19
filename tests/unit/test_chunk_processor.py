import pytest
from src.processing.chunk_processor import ChunkProcessor
from src.processing.text_splitter import TextSplitter
from unittest.mock import MagicMock, patch # Added patch import

@pytest.fixture
def chunk_processor():
    return ChunkProcessor(chunk_size=10, chunk_overlap=2)

@pytest.fixture
def mock_text_splitter():
    # Mock the TextSplitter to control its behavior
    mock = MagicMock(spec=TextSplitter)
    # Configure split_text to return predefined chunks
    mock.split_text.return_value = ["chunk1", "chunk2", "chunk3"]
    # Configure create_documents to return predefined document objects
    mock.create_documents.return_value = [
        MagicMock(page_content="doc_chunk1", metadata={"id": 1}),
        MagicMock(page_content="doc_chunk2", metadata={"id": 2})
    ]
    # Also mock the chunk_size and chunk_overlap attributes
    mock.chunk_size = 10
    mock.chunk_overlap = 2
    return mock

def test_chunk_processor_initialization():
    processor = ChunkProcessor(chunk_size=50, chunk_overlap=10)
    assert isinstance(processor.text_splitter, TextSplitter)
    # Assert directly on the properties of the TextSplitter instance
    assert processor.text_splitter.chunk_size == 50
    assert processor.text_splitter.chunk_overlap == 10

def test_process_text_into_chunks_empty_text(chunk_processor):
    chunks = chunk_processor.process_text_into_chunks("")
    assert chunks == []

def test_process_text_into_chunks_single_chunk(chunk_processor):
    text = "short text"
    # Assuming TextSplitter returns the text as a single chunk if it's small enough
    with patch('src.processing.text_splitter.TextSplitter.split_text', return_value=[text]):
        chunks = chunk_processor.process_text_into_chunks(text)
        assert chunks == [text]

def test_process_text_into_chunks_multiple_chunks(mock_text_splitter):
    # Instantiate ChunkProcessor, but then replace its text_splitter with our mock
    processor = ChunkProcessor()
    processor.text_splitter = mock_text_splitter
    
    text = "This is a longer text that needs to be split into multiple parts."
    chunks = processor.process_text_into_chunks(text)
    
    assert chunks == ["chunk1", "chunk2", "chunk3"]
    mock_text_splitter.split_text.assert_called_once_with(text)

def test_process_documents_into_chunks_empty_texts(chunk_processor):
    chunks = chunk_processor.process_documents_into_chunks([])
    assert chunks == []

def test_process_documents_into_chunks_with_metadata(mock_text_splitter):
    processor = ChunkProcessor()
    processor.text_splitter = mock_text_splitter

    texts = ["doc text 1", "doc text 2"]
    metadatas = [{"source": "a"}, {"source": "b"}]
    
    docs = processor.process_documents_into_chunks(texts, metadatas)
    
    assert len(docs) == 2
    assert docs[0].page_content == "doc_chunk1"
    assert docs[0].metadata == {"id": 1}
    assert docs[1].page_content == "doc_chunk2"
    assert docs[1].metadata == {"id": 2}
    mock_text_splitter.create_documents.assert_called_once_with(texts, metadatas)

def test_process_documents_into_chunks_no_metadata(mock_text_splitter):
    processor = ChunkProcessor()
    processor.text_splitter = mock_text_splitter

    texts = ["doc text 1", "doc text 2"]
    
    docs = processor.process_documents_into_chunks(texts)
    
    assert len(docs) == 2
    assert docs[0].page_content == "doc_chunk1"
    assert docs[0].metadata == {"id": 1}
    assert docs[1].page_content == "doc_chunk2"
    assert docs[1].metadata == {"id": 2}
    mock_text_splitter.create_documents.assert_called_once_with(texts, None)