# Tasks for RAG Ingestion Pipeline

## Feature Name: RAG Ingestion Pipeline

### Phase 1: Setup (Project Initialization)

- [ ] T001 Create project structure as defined in `plan.md` in `src/`
- [ ] T002 Create `requirements.txt` with initial dependencies: `cohere`, `qdrant-client`, `requests`, `beautifulsoup4`, `langchain_text_splitters` in `.`
- [ ] T003 Configure `pytest` for unit, integration, and e2e tests in `pyproject.toml` or `pytest.ini`
- [ ] T004 Create `.env.example` with placeholders for `COHERE_API_KEY`, `QDRANT_API_KEY`, `QDRANT_CLUSTER_URL`, `DOCUSAURUS_BASE_URL`, `QDRANT_COLLECTION_NAME` in `.`

### Phase 2: Foundational (Blocking Prerequisites)

- [ ] T005 Implement a shared utility for loading environment variables from `.env` in `src/utils/env_loader.py`
- [ ] T006 [P] Implement Qdrant client initialization and basic collection management (create/get/delete) in `src/storage/qdrant_client.py`
- [ ] T007 [P] Implement Cohere client initialization in `src/embedding/cohere_client.py`
- [ ] T008 [P] Implement `TextSplitter` utility using `langchain_text_splitters` for token-aware chunking in `src/processing/text_splitter.py`

### Phase 3: User Story 1 - Pipeline Execution for Initial Indexing [US1]

*   **Goal**: As an AI engineer, I want to execute a single script that ingests the entire live Docusaurus book, processes its content, and populates a Qdrant vector database with embeddings and metadata, so that the book's knowledge is ready for a RAG chatbot to query.
*   **Independent Test Criteria**: The script can be run on a deployed Docusaurus site. The test passes if the Qdrant database is populated with vectors and the correct metadata (source URL, chapter, etc.) corresponding to the book's content.

- [ ] T009 [US1] Implement web crawler to discover all public, crawlable pages of a Docusaurus website in `src/crawler/docusaurus_crawler.py`
- [ ] T010 [US1] Implement content extractor to get main textual content from HTML, excluding boilerplate, in `src/crawler/content_extractor.py`
- [ ] T011 [US1] Implement text cleaner to remove excessive whitespace and non-content elements in `src/processing/text_cleaner.py`
- [ ] T012 [US1] Integrate `TextSplitter` to chunk cleaned text into overlapping segments in `src/processing/chunk_processor.py`
- [ ] T013 [US1] Implement embedding generation for text chunks using Cohere client in `src/embedding/embedding_generator.py`
- [ ] T014 [US1] Implement metadata extraction (source_url, module/chapter, section/heading) during content processing in `src/processing/metadata_extractor.py`
- [ ] T015 [US1] Implement storage of vectors and metadata in Qdrant using Qdrant client in `src/storage/vector_store_manager.py`
- [ ] T016 [US1] Develop main CLI entry point for `ingest` command orchestrating crawl -> extract -> clean -> chunk -> embed -> store workflow in `src/cli/main.py`
- [ ] T017 [US1] Create unit tests for `docusaurus_crawler.py` in `tests/unit/test_docusaurus_crawler.py`
- [ ] T018 [US1] Create unit tests for `content_extractor.py` in `tests/unit/test_content_extractor.py`
- [ ] T019 [US1] Create unit tests for `text_cleaner.py` in `tests/unit/test_text_cleaner.py`
- [ ] T020 [US1] Create unit tests for `chunk_processor.py` in `tests/unit/test_chunk_processor.py`
- [ ] T021 [US1] Create unit tests for `embedding_generator.py` in `tests/unit/test_embedding_generator.py`
- [ ] T022 [US1] Create unit tests for `vector_store_manager.py` in `tests/unit/test_vector_store_manager.py`
- [ ] T023 [US1] Create integration tests for the full ingestion pipeline (mocking external APIs) in `tests/integration/test_ingestion_pipeline.py`
- [ ] T024 [US1] Create end-to-end test to run ingestion against a real Docusaurus site and verify Qdrant population in `tests/e2e/test_e2e_ingestion.py`

### Phase 4: User Story 2 - Re-indexing for Content Updates [US2]

*   **Goal**: As a developer, I want to be able to re-run the ingestion pipeline to update the vector store with the latest content from the book, so that the RAG chatbot can answer questions based on the most current information.
*   **Independent Test Criteria**: After a documented change is made to the live book, the re-indexing process can be run. The test passes if the updated content is reflected in the vector store and old content is appropriately handled.

- [ ] T025 [US2] Extend `vector_store_manager.py` to support update-safe ingestion (e.g., upsert functionality based on `source_url` + content hash) in `src/storage/vector_store_manager.py`
- [ ] T026 [US2] Modify `main.py` CLI to support an "update" mode or flag for re-indexing in `src/cli/main.py`
- [ ] T027 [US2] Create integration tests for the update functionality in `tests/integration/test_update_pipeline.py`
- [ ] T028 [US2] Create end-to-end test for re-indexing scenario in `tests/e2e/test_e2e_reindexing.py`

### Final Phase: Polish & Cross-Cutting Concerns

- [ ] T029 Add comprehensive documentation and usage instructions to a `README.md` in `src/` directory.
- [ ] T030 Implement proper logging across all modules for better observability in `src/utils/logger.py`
- [ ] T031 Review and refine error handling mechanisms across the pipeline for robustness.
- [ ] T032 Ensure all code adheres to PEP 8 style guidelines.

## Dependencies

User Story 1 (P1) must be completed before User Story 2 (P2).
- US1: Pipeline Execution for Initial Indexing
- US2: Re-indexing for Content Updates

## Parallel Execution Examples

- Within Phase 1 (Setup), tasks T001-T004 can be executed in parallel once the initial project structure is defined.
- Within Phase 2 (Foundational), tasks T006, T007, and T008 can be developed in parallel as they represent independent foundational components.
- Within Phase 3 (User Story 1), tasks T009 (crawler), T010 (extractor), T011 (cleaner), T012 (chunker), T013 (embedding), T014 (metadata) and T015 (storage) can be developed by different engineers working on their respective modules once interfaces are agreed upon. Tests (T017-T022) can also run in parallel with their corresponding implementation. Integration and E2E tests (T023, T024) are dependent on their respective components.
- Within Phase 4 (User Story 2), tasks T025 (update logic), T026 (CLI update mode) can be developed in parallel, followed by tests T027 and T028.

## Implementation Strategy

We will follow an MVP-first approach, focusing on delivering User Story 1 (Pipeline Execution for Initial Indexing) as the Minimum Viable Product. This ensures the core functionality of ingesting and populating the vector store is working end-to-end. Subsequent iterations will then focus on User Story 2 (Re-indexing for Content Updates) and other polish tasks. Each user story phase is designed to be independently testable, allowing for incremental delivery and verification.