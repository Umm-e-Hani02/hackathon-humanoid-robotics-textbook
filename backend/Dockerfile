FROM python:3.10-slim

WORKDIR /app

# Copy the entire project (from project root where both src and backend exist)
COPY . .

# Install requirements from the backend directory
WORKDIR /app/backend
COPY ./requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

EXPOSE 8000

# Set the working directory for runtime
WORKDIR /app/backend

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "${PORT:-8000}"]