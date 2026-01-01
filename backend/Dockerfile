FROM python:3.10-slim

WORKDIR /app

# Copy only the backend folder if context is backend/
# OR copy everything if context is project root.
# To be safe for Railway building from subfolder:
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the files from the current context
COPY . .

# Railway uses 8080
EXPOSE 8080

# Ensure project root is in PYTHONPATH
ENV PYTHONPATH=/app

# IMPORTANT: run from project root
CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "8080"]
