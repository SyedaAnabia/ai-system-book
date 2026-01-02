---
title: Physical AI Course Backend
emoji: 🤖
colorFrom: blue
colorTo: red
sdk: docker
pinned: false
---

# Physical AI Course Backend

This Hugging Face Space hosts the backend API for the Physical AI Course chatbot.

## Endpoints

- `/api/chat` - Chat endpoint
- `/api/auth/register` - User registration
- `/api/auth/login` - User login
- `/api/documents/upload` - Document upload
- `/api/documents/search` - Document search
- `/health` - Health check

## Environment Variables

You need to set the following environment variables:

- `DATABASE_URL`: PostgreSQL database connection string
- `GROQ_API_KEY`: Your Groq API key
- `QDRANT_URL`: Qdrant cloud instance URL
- `QDRANT_API_KEY`: Qdrant API key