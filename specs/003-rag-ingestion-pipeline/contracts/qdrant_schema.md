# Qdrant Collection Schema Contract

This document outlines the expected schema for the Qdrant collection used to store the vectorized book content.

## Collection Name

The collection name should be configurable, but a recommended default could be `physical-ai-book-rag`.

## Vector Parameters

*   **`size`**: Corresponds to the output dimension of the Cohere embedding model used (e.g., 1024 for `embed-english-v3.0`). This should be dynamically retrieved or configured based on the chosen Cohere model.
*   **`distance`**: `Cosine` (cosine similarity is a common and effective metric for text embeddings).

## Payload Schema (Metadata)

The payload for each point in Qdrant will store the metadata associated with the `Book Content Chunk`. This aligns directly with the `metadata` attributes defined in the `Vector Record` entity of the `data-model.md`.

### Required Fields

*   **`source_url`** (string): The URL of the Docusaurus page. Critical for tracing back to the original content.
*   **`text_content`** (string): The original text content of the chunk. Storing this allows for direct retrieval and display without needing to re-fetch from the source URL.

### Optional Fields (for enhanced context and filtering)

*   **`module`** (string): The top-level grouping (e.g., "Module 1").
*   **`chapter`** (string): A more granular grouping within a module or an alias for module.
*   **`section`** (string): A major section or heading within a page.
*   **`heading`** (string): A more granular grouping within a section or an alias for section.
*   **`original_text_start`** (integer): The starting character index of the chunk within the full page text.
*   **`original_text_end`** (integer): The ending character index of the chunk within the full page text.

## Example Qdrant Point Structure

```json
{
  "id": "uuid_or_sequential_id",
  "vector": [0.1, 0.2, ..., 0.9], // Cohere embedding vector
  "payload": {
    "source_url": "https://example.com/docs/module1/ros2-basics",
    "text_content": "ROS 2 is the second generation of the Robot Operating System...",
    "module": "Module 1",
    "chapter": "ROS2 Basics",
    "section": "Introduction to ROS 2",
    "heading": "What is ROS 2?",
    "original_text_start": 1234,
    "original_text_end": 1567
  }
}
```
