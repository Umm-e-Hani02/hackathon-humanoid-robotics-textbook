FROM python:3.10-slim

WORKDIR /app

# Simple, robust Dockerfile for Railway
WORKDIR /app

# Copy all files from the current build context
# If Railway is at root, it copies everything (including src and backend)
# If Railway is at /backend, it copies only backend contents
COPY . .

# Install dependencies from whichever requirements.txt is available in the current context
RUN if [ -f backend/requirements.txt ]; then \
        pip install --no-cache-dir -r backend/requirements.txt; \
    elif [ -f requirements.txt ]; then \
        pip install --no-cache-dir -r requirements.txt; \
    fi

# Railway uses 8080
EXPOSE 8080

# The PYTHONPATH is the key to fixing the 'src' import error.
# We ensure Python looks in the root (/app) for the 'src' package.
ENV PYTHONPATH=/app

# Start the application
# We try to run it from backend.main, or just main if the folder structure is flat
CMD ["sh", "-c", "if [ -f backend/main.py ]; then uvicorn backend.main:app --host 0.0.0.0 --port 8080; else uvicorn main:app --host 0.0.0.0 --port 8080; fi"]
