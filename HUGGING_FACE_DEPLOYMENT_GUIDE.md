# Deploying to Hugging Face Spaces

This guide explains how to deploy your backend to Hugging Face Spaces.

## Prerequisites

- A Hugging Face account
- Your backend code repository
- Required environment variables:
  - `DATABASE_URL`: PostgreSQL database connection string
  - `GROQ_API_KEY`: Your Groq API key
  - `QDRANT_URL`: Qdrant cloud instance URL
  - `QDRANT_API_KEY`: Qdrant API key

## Deployment Steps

### 1. Prepare Your Code

Your repository already contains all necessary files:
- `app.py` - The main application file
- `requirements.txt` - Python dependencies
- `Dockerfile` - Container configuration
- `README.md` - Space configuration

### 2. Deploy to Hugging Face Spaces

1. Go to [huggingface.co/spaces](https://huggingface.co/spaces)
2. Click "Create new Space"
3. Choose:
   - SDK: Docker
   - Hardware: CPU (or higher if needed)
   - Visibility: Public or Private
4. Add your repository:
   - Option A: Link your GitHub repository
   - Option B: Upload the files directly
5. Add the environment variables in the Space settings:
   - DATABASE_URL
   - GROQ_API_KEY
   - QDRANT_URL
   - QDRANT_API_KEY

### 3. Update Frontend Configuration

After deploying your backend, update the frontend to use the Hugging Face Space URL:

1. In `src/components/HomepageFeatures/FloatingChatbot.tsx`, update the `GITHUB_PAGES_API_BASE_URL` constant:
   ```javascript
   const GITHUB_PAGES_API_BASE_URL = 'https://your-username-your-space-name.hf.space';
   ```

2. In `.env.production`, update:
   ```
   REACT_APP_API_BASE_URL=https://your-username-your-space-name.hf.space
   API_URL=https://your-username-your-space-name.hf.space/api/documents/upload
   ```

### 4. Environment Variables Explained

- `DATABASE_URL`: Connection string for your PostgreSQL database (e.g., from Supabase, Aiven, or other providers)
- `GROQ_API_KEY`: API key from https://console.groq.com/keys
- `QDRANT_URL`: URL for your Qdrant vector database (e.g., from Qdrant Cloud)
- `QDRANT_API_KEY`: API key for your Qdrant instance

### 5. Testing the Deployment

1. After deployment, test the health endpoint: `https://your-username-your-space-name.hf.space/health`
2. Verify that the API endpoints are accessible
3. Test the chat functionality with the frontend

### 6. Important Considerations

⚠️ **Limitations of Hugging Face Spaces for Backend APIs:**

- **Sleeping**: Spaces sleep when inactive, causing slow response times
- **Resource Limits**: Limited CPU and RAM
- **Not Ideal for APIs**: Spaces are designed for UIs, not API backends
- **Rate Limits**: May have request limits

For a more reliable solution, consider using:
- Railway (has free tier)
- Google Cloud Run (has free tier)
- Fly.io (has free tier)
- Koyeb (has free tier)

### 7. Troubleshooting

- If you encounter CORS errors, ensure your backend's `allow_origins` includes your frontend's URL
- Check the Hugging Face Space logs for any runtime errors
- Verify all environment variables are correctly set
- Make sure your Qdrant and database connections are properly configured