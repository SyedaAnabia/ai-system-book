# Backend Deployment Guide

This guide explains how to deploy the backend service to a cloud platform (specifically Render.com, which is already referenced in the code).

## Prerequisites

- A Render.com account
- Your backend code repository
- Required environment variables:
  - `DATABASE_URL`: PostgreSQL database connection string
  - `GROQ_API_KEY`: Your Groq API key
  - `QDRANT_URL`: Qdrant cloud instance URL
  - `QDRANT_API_KEY`: Qdrant API key

## Deployment Steps

### 1. Prepare Your Code

1. Ensure your `requirements.txt` file includes all necessary dependencies:
   ```
   fastapi
   uvicorn[standard]
   python-dotenv
   psycopg2-binary
   bcrypt
   httpx
   qdrant-client
   sentence-transformers
   torch
   transformers
   ```

2. Make sure you have a `Procfile` in your repository root with the following content:
   ```
   web: uvicorn auth-backend.main:app --host 0.0.0.0 --port $PORT
   ```

### 2. Deploy to Render

1. Push your code to a GitHub repository
2. Go to https://render.com and sign up/in
3. Click "New +" and select "Web Service"
4. Connect your GitHub repository
5. Select the branch you want to deploy
6. Choose "Python" as the environment
7. Set the build command (if needed): `pip install -r requirements.txt`
8. Set the start command: `uvicorn auth-backend.main:app --host 0.0.0.0 --port $PORT`
9. Add the required environment variables in the dashboard
10. Click "Create Web Service"

### 3. Update Frontend Configuration

After deploying your backend, update the frontend to use the deployed backend URL:

1. In `src/components/HomepageFeatures/FloatingChatbot.tsx`, update the `GITHUB_PAGES_API_BASE_URL` constant:
   ```javascript
   const GITHUB_PAGES_API_BASE_URL = 'https://your-render-service-name.onrender.com';
   ```

2. In `.env.production`, update:
   ```
   REACT_APP_API_BASE_URL=https://your-render-service-name.onrender.com
   API_URL=https://your-render-service-name.onrender.com/api/documents/upload
   ```

### 4. Environment Variables Explained

- `DATABASE_URL`: Connection string for your PostgreSQL database (e.g., from Render's free PostgreSQL service or other providers)
- `GROQ_API_KEY`: API key from https://console.groq.com/keys
- `QDRANT_URL`: URL for your Qdrant vector database (e.g., from Qdrant Cloud)
- `QDRANT_API_KEY`: API key for your Qdrant instance

### 5. Testing the Deployment

1. After deployment, test the health endpoint: `https://your-service-name.onrender.com/health`
2. Verify that the API endpoints are accessible
3. Test the chat functionality with the frontend

### 6. Troubleshooting

- If you encounter CORS errors, ensure your backend's `allow_origins` includes your frontend's URL
- Check the Render logs for any runtime errors
- Verify all environment variables are correctly set
- Make sure your Qdrant and database connections are properly configured

## Alternative Cloud Platforms

You can also deploy to other platforms like:

- **Railway**: Similar to Render, with easy environment variable management
- **Heroku**: Well-documented Python deployment process
- **Google Cloud Run**: Container-based deployment
- **AWS Elastic Beanstalk**: Traditional deployment option

Each platform will have slightly different configuration steps, but the core requirements remain the same: environment variables, proper start commands, and correct CORS settings.