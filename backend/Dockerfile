FROM python:3.10-slim

WORKDIR /app

# Build from project root context
WORKDIR /app

# Copy specifically what we need
COPY requirements.txt .
COPY backend/requirements.txt ./backend/requirements.txt
RUN pip install --no-cache-dir -r backend/requirements.txt

# Copy all source and backend files
COPY src/ ./src/
COPY backend/ ./backend/

# Railway uses 8080
EXPOSE 8080

# Set PYTHONPATH to include the /app directory so 'src' is found
ENV PYTHONPATH=/app

# Run from the context of /app
CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "8080"]
