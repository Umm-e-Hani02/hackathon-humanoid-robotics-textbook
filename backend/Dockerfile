FROM python:3.10-slim

WORKDIR /app

# Build from project root context
WORKDIR /app

# Copy specifically what we need
# Using wildcards/optional paths helps with different build environments
COPY requirements.txt .
COPY backend/requirements.txt* ./backend/
RUN if [ -f backend/requirements.txt ]; then pip install --no-cache-dir -r backend/requirements.txt; else pip install --no-cache-dir -r requirements.txt; fi

# Copy all source and backend files
# If we are in the root, /src and /backend exist
# If we are in backend/, we need to handle it gracefully
COPY . .

# Railway uses 8080
EXPOSE 8080

# Set PYTHONPATH to include the current directory so 'src' can be found
# We add both /app and /app/backend to be extremely safe
ENV PYTHONPATH=/app:/app/backend

# Run the app
# Use a string command to allow path searching
CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "8080"]
