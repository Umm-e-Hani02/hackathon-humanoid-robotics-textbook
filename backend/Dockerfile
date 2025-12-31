FROM python:3.10-slim

WORKDIR /app

# Copy requirements first for better caching
COPY ../requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy src directory from parent
COPY ../src ./src
# Copy backend directory contents
COPY . .

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "${PORT:-8000}"]