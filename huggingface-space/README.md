---
title: Physical AI & Humanoid Robotics Chatbot
emoji: 🤖
colorFrom: purple
colorTo: blue
sdk: docker
pinned: false
license: mit
---

# Physical AI & Humanoid Robotics Chatbot

This is a RAG (Retrieval Augmented Generation) powered chatbot for the Physical AI & Humanoid Robotics course. It uses Qdrant for vector storage and Groq for LLM inference.

## API Endpoints

- `/api/chat` - Main chat endpoint
- `/api/auth/register` - User registration
- `/api/auth/login` - User login
- `/api/documents/upload` - Document upload for RAG
- `/api/documents/search` - Document search
- `/health` - Health check

## Environment Variables Required

- `GROQ_API_KEY` - API key for Groq LLM service
- `DATABASE_URL` - Database connection string
- `QDRANT_URL` - Qdrant vector database URL
- `QDRANT_API_KEY` - Qdrant API key (if using cloud)