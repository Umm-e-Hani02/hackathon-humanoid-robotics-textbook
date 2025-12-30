---

description: "Task list for RAG Agent for Chatbot feature implementation"
---

# Tasks: RAG Agent for Chatbot

**Input**: Design documents from `/specs/001-rag-chatbot-agent/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/openapi.yaml

**Tests**: The feature specification (spec.md) and plan.md imply a need for local testing and verification. Test tasks are included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `src/` at repository root
- Paths shown below assume project structure from plan.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create `src/rag_agent` directory and `__init__.py` file.
- [X] T002 Add `fastapi`, `openai-agents`, `qdrant-client`, `cohere` packages to `requirements.txt`.
- [X] T003 Verify `backend/main.py` exists and is ready for endpoint additions.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Implement environment variable loading in `src/utils/env_loader.py` for `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`.
- [X] T005 Create `src/storage/qdrant_client.py` for Qdrant connection and basic operations (`get_client`, `search`).
- [X] T006 Create `src/embedding/cohere_client.py` for Cohere embedding generation.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Chatbot Query for Book Data (Priority: P1) 🎯 MVP

**Goal**: A user asks a question to the chatbot, and the chatbot retrieves and presents relevant information from the embedded book data in Qdrant.

**Independent Test**: Can be fully tested by providing a user query to the chatbot and verifying the retrieved and presented answer against the source book data.

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T007 [US1] Create `tests/unit/test_rag_agent.py` for RAG agent unit tests.
- [X] T008 [US1] Add test case to `tests/unit/test_rag_agent.py` to simulate a user query and verify relevant content retrieval from a mock Qdrant client.
- [X] T009 [US1] Add test case to `tests/unit/test_rag_agent.py` to simulate an out-of-scope query and verify graceful no-content response from a mock Qdrant client.

### Implementation for User Story 1

- [X] T010 [US1] Implement core RAG agent logic in `src/rag_agent/agent.py` using OpenAI Agents SDK, including agent initialization, retrieval tool definition (Qdrant), and query processing.
- [X] T011 [US1] Implement FastAPI service logic in `src/rag_agent/service.py` to interface with `agent.py`, defining the `/chat` endpoint (`POST`), processing `UserQuery`, invoking the RAG agent, and constructing `ChatbotResponse`.
- [X] T012 [US1] Integrate `src/rag_agent/service.py` into `backend/main.py` by adding the `/chat` endpoint router.
- [X] T013 [US1] Implement logging within `src/rag_agent/agent.py` and `src/rag_agent/service.py` for queries, retrieved chunks, and responses.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T014 Review and refine error handling across `src/rag_agent/` and `backend/main.py`.
- [ ] T015 Ensure all sensitive information (API keys) are loaded securely via `env_loader.py` and not hardcoded.
- [ ] T016 Update `README.md` in `physical-ai-and-humanoid-robots` with instructions on how to run the RAG agent locally.
- [ ] T017 Run end-to-end integration tests (manual or automated) for the RAG agent.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Add test case to tests/unit/test_rag_agent.py to simulate a user query and verify relevant content retrieval from a mock Qdrant client."
Task: "Add test case to tests/unit/test_rag_agent.py to simulate an out-of-scope query and verify graceful no-content response from a mock Qdrant client."
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence