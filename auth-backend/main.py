from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, EmailStr
import psycopg2
from psycopg2.extras import RealDictCursor
import bcrypt
import os
from dotenv import load_dotenv
from typing import List, Optional
import json
import httpx
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from sentence_transformers import SentenceTransformer
import uuid
from contextlib import asynccontextmanager

load_dotenv()

# Environment Variables
DATABASE_URL = os.getenv("DATABASE_URL")
GROQ_API_KEY = os.getenv("GROQ_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Global variables
qdrant_client = None
embedding_model = None
COLLECTION_NAME = "physical_ai_book"

# Lifespan context manager (replaces on_event)
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    global qdrant_client, embedding_model

    print("🚀 Starting up...")

    # Initialize Qdrant Client
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        timeout=30
    )

    # Initialize Sentence Transformer
    print("📥 Loading embedding model...")
    embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

    # Create collection if not exists
    try:
        collections = qdrant_client.get_collections().collections
        collection_exists = any(col.name == COLLECTION_NAME for col in collections)

        if not collection_exists:
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=384,
                    distance=Distance.COSINE
                )
            )
            print(f"✅ Created Qdrant collection: {COLLECTION_NAME}")
        else:
            print(f"✅ Collection {COLLECTION_NAME} already exists")
    except Exception as e:
        print(f"⚠️ Qdrant initialization warning: {str(e)}")

    print("✅ Startup complete!")

    yield

    # Shutdown
    print("👋 Shutting down...")

app = FastAPI(
    title="Physical AI Course API - RAG Enabled",
    lifespan=lifespan
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "https://yourdomain.github.io"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

def get_db_connection():
    try:
        conn = psycopg2.connect(DATABASE_URL)
        return conn
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Database connection failed: {str(e)}")

# Pydantic Models
class UserRegistration(BaseModel):
    name: str
    email: EmailStr
    password: str
    softwareExperience: str
    programmingLanguages: List[str]
    hasGPU: bool
    gpuType: Optional[str] = ""
    rosExperience: str
    roboticsProjects: bool
    roboticsDetails: Optional[str] = ""
    learningGoals: str
    hardwareAccess: List[str]

class UserLogin(BaseModel):
    email: EmailStr
    password: str

class ChatMessage(BaseModel):
    message: str
    conversation_id: Optional[str] = None
    selected_text: Optional[str] = None
    user_id: Optional[int] = None

class DocumentUpload(BaseModel):
    text: str
    metadata: Optional[dict] = {}

# Helper Functions
def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """Split text into overlapping chunks"""
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]

        if end < len(text):
            last_period = chunk.rfind('.')
            last_newline = chunk.rfind('\n')
            break_point = max(last_period, last_newline)
            if break_point > chunk_size * 0.5:
                chunk = chunk[:break_point + 1]
                end = start + break_point + 1

        chunks.append(chunk.strip())
        start = end - overlap

    return [c for c in chunks if len(c) > 50]

async def retrieve_context(query: str, top_k: int = 5) -> str:
    """Retrieve relevant context from Qdrant"""
    try:
        # Generate query embedding
        query_vector = embedding_model.encode(query).tolist()

        # Search in Qdrant (updated method)
        search_results = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_vector,
            limit=top_k,
            score_threshold=0.3
        ).points

        if not search_results:
            return "No relevant context found in the book."

        # Combine contexts with scores
        contexts = []
        for result in search_results:
            score = result.score
            text = result.payload.get('text', '')
            source = result.payload.get('source', 'Unknown')
            contexts.append(f"[Source: {source} | Relevance: {score:.2f}]\n{text}")

        return "\n\n---\n\n".join(contexts)

    except Exception as e:
        print(f"⚠️ Retrieval error: {str(e)}")
        return "Error retrieving context from knowledge base."

# API Endpoints
@app.get("/")
async def root():
    try:
        collections = qdrant_client.get_collections()
        vector_count = 0
        for col in collections.collections:
            if col.name == COLLECTION_NAME:
                collection_info = qdrant_client.get_collection(COLLECTION_NAME)
                vector_count = collection_info.points_count
    except:
        vector_count = 0

    return {
        "message": "Physical AI Course API - RAG Enabled 🚀",
        "version": "2.0",
        "features": "Groq LLM + Qdrant Cloud + RAG",
        "vectors_indexed": vector_count,
        "endpoints": {
            "register": "/api/auth/register",
            "login": "/api/auth/login",
            "chat": "/api/chat",
            "upload": "/api/documents/upload",
            "search": "/api/documents/search"
        }
    }

@app.post("/api/auth/register")
async def register(user: UserRegistration):
    conn = None
    cur = None
    try:
        conn = get_db_connection()
        cur = conn.cursor(cursor_factory=RealDictCursor)

        cur.execute("SELECT id FROM users WHERE email = %s", (user.email,))
        if cur.fetchone():
            raise HTTPException(status_code=400, detail="Email already registered")

        hashed_password = bcrypt.hashpw(user.password.encode('utf-8'), bcrypt.gensalt())

        cur.execute("""
            INSERT INTO users (
                name, email, password, software_experience,
                programming_languages, has_gpu, gpu_type,
                ros_experience, robotics_projects, robotics_details,
                learning_goals, hardware_access, created_at
            )
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, NOW())
            RETURNING id, name, email
        """, (
            user.name,
            user.email,
            hashed_password.decode('utf-8'),
            user.softwareExperience,
            json.dumps(user.programmingLanguages),
            user.hasGPU,
            user.gpuType,
            user.rosExperience,
            user.roboticsProjects,
            user.roboticsDetails,
            user.learningGoals,
            json.dumps(user.hardwareAccess)
        ))

        new_user = cur.fetchone()
        conn.commit()

        return {
            "message": "Registration successful! 🎉",
            "user": {
                "id": new_user['id'],
                "name": new_user['name'],
                "email": new_user['email']
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        if conn:
            conn.rollback()
        raise HTTPException(status_code=500, detail=f"Registration failed: {str(e)}")
    finally:
        if cur:
            cur.close()
        if conn:
            conn.close()

@app.post("/api/auth/login")
async def login(credentials: UserLogin):
    conn = None
    cur = None
    try:
        conn = get_db_connection()
        cur = conn.cursor(cursor_factory=RealDictCursor)

        cur.execute("""
            SELECT id, name, email, password, software_experience,
                   programming_languages, has_gpu, gpu_type, ros_experience
            FROM users WHERE email = %s
        """, (credentials.email,))

        user = cur.fetchone()

        if not user:
            raise HTTPException(status_code=401, detail="Invalid email or password")

        if not bcrypt.checkpw(credentials.password.encode('utf-8'), user['password'].encode('utf-8')):
            raise HTTPException(status_code=401, detail="Invalid email or password")

        return {
            "message": "Login successful! ✅",
            "user": {
                "id": user['id'],
                "name": user['name'],
                "email": user['email'],
                "softwareExperience": user['software_experience'],
                "programmingLanguages": json.loads(user['programming_languages']) if user['programming_languages'] else [],
                "hasGPU": user['has_gpu'],
                "gpuType": user['gpu_type'],
                "rosExperience": user['ros_experience']
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Login failed: {str(e)}")
    finally:
        if cur:
            cur.close()
        if conn:
            conn.close()

@app.post("/api/chat")
async def chat(message: ChatMessage):
    """RAG Chatbot with Qdrant + Groq"""
    try:
        if not GROQ_API_KEY:
            raise HTTPException(status_code=500, detail="Groq API key not configured")

        # 🎯 RETRIEVE relevant context from Qdrant
        context = await retrieve_context(message.message, top_k=5)

        # 🤖 GENERATE response using Groq
        async with httpx.AsyncClient() as client:
            response = await client.post(
                "https://api.groq.com/openai/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {GROQ_API_KEY}",
                    "Content-Type": "application/json"
                },
                json={
                    "model": "llama-3.3-70b-versatile",  # ✅ Updated to supported model
                    "messages": [
                        {
                            "role": "system",
                            "content": f"""You are a helpful AI assistant for a Physical AI & Humanoid Robotics course.

Use the following context from the course book to answer questions. If the context doesn't contain the answer, say so clearly.

Context from book:
{context}

Guidelines:
- Answer based on the context provided
- Be specific and cite relevant sections when available
- If uncertain or context is insufficient, acknowledge it
- Keep responses clear and educational
- If no relevant context is found, provide a general answer but mention the limitation"""
                        },
                        {
                            "role": "user",
                            "content": message.message
                        }
                    ],
                    "temperature": 0.3,
                    "max_tokens": 1024,
                    "top_p": 0.9
                },
                timeout=30.0
            )

            if response.status_code != 200:
                print(f"Groq API error: {response.status_code} - {response.text}")  # Log the error
                raise HTTPException(status_code=500, detail=f"Groq API error: {response.text}")

            result = response.json()

            # Check if the expected structure exists
            if 'choices' not in result or not result['choices']:
                print(f"Unexpected Groq response structure: {result}")
                raise HTTPException(status_code=500, detail="Unexpected response from Groq API")

            ai_response = result['choices'][0]['message']['content']

        # Generate conversation ID if not provided
        if not message.conversation_id:
            conversation_id = f"conv_{message.user_id}_{uuid.uuid4().hex[:8]}" if message.user_id else f"conv_{uuid.uuid4().hex[:8]}"
        else:
            conversation_id = message.conversation_id

        # Try to save to database, but don't fail if database is unavailable
        try:
            conn = get_db_connection()
            cur = conn.cursor(cursor_factory=RealDictCursor)

            # Create conversation if it doesn't exist
            if not message.conversation_id:
                cur.execute("""
                    INSERT INTO conversations (user_id, conversation_id, created_at)
                    VALUES (%s, %s, NOW())
                    RETURNING id, conversation_id
                """, (message.user_id, conversation_id))
                conv = cur.fetchone()
                conversation_id = conv['conversation_id']
            else:
                conversation_id = message.conversation_id

            # Insert messages
            cur.execute("""
                INSERT INTO messages (conversation_id, role, content, created_at)
                SELECT c.id, 'user', %s, NOW()
                FROM conversations c WHERE c.conversation_id = %s
            """, (message.message, conversation_id))

            cur.execute("""
                INSERT INTO messages (conversation_id, role, content, created_at)
                SELECT c.id, 'assistant', %s, NOW()
                FROM conversations c WHERE c.conversation_id = %s
            """, (ai_response, conversation_id))

            conn.commit()
            cur.close()
            conn.close()
        except Exception as db_error:
            print(f"Database error (non-fatal): {str(db_error)}")
            # Continue without saving to database

        return {
            "response": ai_response,
            "conversation_id": conversation_id,
            "model": "llama-3.3-70b-versatile (Groq)",  # ✅ Updated model name
            "context_found": "No relevant" not in context
        }

    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        print(f"Chat endpoint error: {str(e)}")  # Log the error
        raise HTTPException(status_code=500, detail=f"Chat failed: {str(e)}")

@app.post("/api/documents/upload")
async def upload_document(doc: DocumentUpload):
    """Upload and index document chunks into Qdrant"""
    try:
        # Chunk the text
        chunks = chunk_text(doc.text, chunk_size=500, overlap=50)

        # Generate embeddings and upload to Qdrant
        points = []
        for i, chunk in enumerate(chunks):
            embedding = embedding_model.encode(chunk).tolist()

            point = PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "text": chunk,
                    "chunk_index": i,
                    **doc.metadata
                }
            )
            points.append(point)

        # Upload to Qdrant
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )

        return {
            "message": "Document indexed successfully! ✅",
            "chunks_created": len(chunks),
            "collection": COLLECTION_NAME
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Upload failed: {str(e)}")

class SearchQuery(BaseModel):
    query: str
    top_k: int = 5

@app.post("/api/documents/search")
async def search_documents(search: SearchQuery):
    """Search for relevant document chunks"""
    try:
        context = await retrieve_context(search.query, top_k=search.top_k)
        return {
            "query": search.query,
            "results": context,
            "top_k": search.top_k
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Search failed: {str(e)}")

@app.get("/health")
async def health():
    try:
        collections = qdrant_client.get_collections()
        qdrant_status = "connected"
    except:
        qdrant_status = "disconnected"

    return {
        "status": "healthy",
        "database": "connected",
        "ai_provider": "Groq (FREE)",
        "vector_db": f"Qdrant ({qdrant_status})",
        "embedding_model": "all-MiniLM-L6-v2"
    }

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", 8000))
    uvicorn.run("main:app", host="0.0.0.0", port=port, reload=True)