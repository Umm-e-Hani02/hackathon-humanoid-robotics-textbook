FROM python:3.10-slim

WORKDIR /app

# Copy full project so src/ and backend/ both exist
COPY . .

# Install backend dependencies
RUN pip install --no-cache-dir -r backend/requirements.txt

# Railway uses 8080
EXPOSE 8080

# IMPORTANT: run from project root
CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "8080"]
