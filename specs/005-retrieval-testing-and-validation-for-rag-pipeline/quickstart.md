# Quickstart for Retrieval Validation

This document describes how to run the retrieval validation script.

## Prerequisites

- Python 3.13
- An environment with the project dependencies installed (`pip install -r requirements.txt`)
- A running Qdrant instance with the book's data

## Running the validation

To run the validation script, execute the following command:

```bash
pytest tests/validation/test_retrieval.py
```

The script will run a series of tests that connect to the Qdrant collection, perform similarity searches for a set of sample queries, and validate the retrieved chunks. The results will be logged to the console.
