import re

class TextCleaner:
    def __init__(self):
        """
        Initializes the TextCleaner.
        """
        pass

    def clean_text(self, text: str) -> str:
        """
        Cleans the input text by removing excessive whitespace and common non-content elements.

        Args:
            text (str): The input text string.

        Returns:
            str: The cleaned text string.
        """
        if not text:
            return ""

        # Remove extra whitespace (multiple spaces, tabs, newlines)
        # Replace multiple newlines with a single space or just remove them, depending on desired output.
        # Here, replace all whitespace sequences with a single space.
        text = re.sub(r'\s+', ' ', text)

        # Remove leading/trailing whitespace
        text = text.strip()

        # Remove common non-content patterns that might survive extraction
        # e.g., bullet points that are not part of a list, specific Docusaurus artifacts
        # This list can be expanded as more patterns are identified during testing.
        non_content_patterns = [
            r'\[edit this page\]',
            r'\u2022',                  # Bullet point character

            r'\[ \]',
            r'\[X\]',
            r'Direct link to heading',
            r'Skip to main content',
            r'Table of Contents',
            r'on this page',
            r'theme-doc-breadcrumbs',
        ]
        for pattern in non_content_patterns:
            text = re.sub(pattern, '', text, flags=re.IGNORECASE)
        
        # Clean up any extra spaces that might result from pattern removal
        text = re.sub(r'\s+', ' ', text).strip()

        return text

if __name__ == "__main__":
    cleaner = TextCleaner()

    # Example 1: Basic cleaning
    text1 = "  This is a   test text.\n  With some\t  extra  whitespace. "
    print("--- Example 1 ---")
    print(f"Original: '{text1}'")
    print(f"Cleaned:  '{cleaner.clean_text(text1)}'")
    # Expected: 'This is a test text. With some extra whitespace.'

    # Example 2: With Docusaurus remnants
    text2 = " # Heading 1 [edit this page] This is content. \u2022 Item 1 [X] Item 2. "
    print("\n--- Example 2 ---")
    print(f"Original: '{text2}'")
    print(f"Cleaned:  '{cleaner.clean_text(text2)}'")
    # Expected: 'Heading 1 This is content. Item 1 Item 2.'

    # Example 3: Empty string
    text3 = ""
    print("\n--- Example 3 ---")
    print(f"Original: '{text3}'")
    print(f"Cleaned:  '{cleaner.clean_text(text3)}'")
    # Expected: ''

    # Example 4: Text with only whitespace
    text4 = "   \n\t  "
    print("\n--- Example 4 ---")
    print(f"Original: '{text4}'")
    print(f"Cleaned:  '{cleaner.clean_text(text4)}'")
    # Expected: ''
