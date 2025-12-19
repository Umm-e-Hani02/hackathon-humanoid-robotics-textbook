import pytest
from src.processing.text_cleaner import TextCleaner

@pytest.fixture
def cleaner():
    return TextCleaner()

def test_clean_text_basic_whitespace(cleaner):
    text1 = "  This is a   test text.\n  With some\t  extra  whitespace. "
    expected_text = "This is a test text. With some extra whitespace."
    assert cleaner.clean_text(text1) == expected_text

def test_clean_text_docusaurus_remnants(cleaner):
    text2 = " # Heading 1 [edit this page] This is content. \u2022 Item 1 [X] Item 2. Direct link to heading"
    expected_text = "Heading 1 This is content. Item 1 Item 2."
    assert cleaner.clean_text(text2) == expected_text

def test_clean_text_empty_string(cleaner):
    text = ""
    expected_text = ""
    assert cleaner.clean_text(text) == expected_text

def test_clean_text_only_whitespace(cleaner):
    text = "   \n\t  "
    expected_text = ""
    assert cleaner.clean_text(text) == expected_text

def test_clean_text_leading_trailing_whitespace(cleaner):
    text = "  leading and trailing  "
    expected_text = "leading and trailing"
    assert cleaner.clean_text(text) == expected_text

def test_clean_text_no_cleaning_needed(cleaner):
    text = "This text needs no cleaning."
    expected_text = "This text needs no cleaning."
    assert cleaner.clean_text(text) == expected_text

def test_clean_text_complex_patterns(cleaner):
    text = (
        "  \tThis is a text. [edit this page] # Some title. \u2022 bullet item.\n"
        "Another line with [ ] checkbox and [X] checked box. "
        "Direct link to heading Table of Contents on this page theme-doc-breadcrumbs"
    )
    expected_text = "This is a text. Some title. bullet item. Another line with checkbox and checked box."
    assert cleaner.clean_text(text) == expected_text

def test_clean_text_with_numbers_and_special_chars(cleaner):
    text = "  123 Test!@#$. %^&*()-+= "
    expected_text = "123 Test!@#$. %^&*()-+="
    assert cleaner.clean_text(text) == expected_text

def test_clean_text_multiple_pattern_removal_leaves_single_space(cleaner):
    text = "Hello [edit this page] World"
    expected_text = "Hello World"
    assert cleaner.clean_text(text) == expected_text
