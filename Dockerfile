FROM python:3.10-slim

# 1. Hugging Face user setup (Permissions issues se bachne ke liye)
RUN useradd -m -u 1000 user
USER user
ENV PATH="/home/user/.local/bin:$PATH"

WORKDIR /app

# 2. Requirements install karein
COPY --chown=user:user requirements.txt .
RUN pip install --no-cache-dir --upgrade -r requirements.txt

# 3. Baaki sara code copy karein
COPY --chown=user:user . .

# 4. Port 7860 lazmi hai (HF isi port ko listen karta hai)
EXPOSE 7860

# 5. App chalane ki command
CMD ["python", "app.py"]