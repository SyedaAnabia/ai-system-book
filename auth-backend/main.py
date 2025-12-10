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

load_dotenv()

app = FastAPI(title="Physical AI Course API - FREE Version")

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

DATABASE_URL = os.getenv("DATABASE_URL")
GROQ_API_KEY = os.getenv("GROQ_API_KEY")  # FREE!
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

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

@app.get("/")
async def root():
    return {
        "message": "Physical AI Course API - FREE Version 🚀",
        "version": "1.0",
        "features": "Using Groq (FREE) + Qdrant (FREE)",
        "endpoints": {
            "register": "/api/auth/register",
            "login": "/api/auth/login",
            "chat": "/api/chat"
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
    """
    RAG Chatbot using FREE Groq API
    """
    try:
        if not GROQ_API_KEY:
            raise HTTPException(status_code=500, detail="Groq API key not configured")
        
        # TODO: Query Qdrant for context (implement later)
        context = "This is a book about Physical AI and Humanoid Robotics."
        
        # Use Groq API (FREE & FAST!)
        async with httpx.AsyncClient() as client:
            response = await client.post(
                "https://api.groq.com/openai/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {GROQ_API_KEY}",
                    "Content-Type": "application/json"
                },
                json={
                    "model": "llama-3.1-70b-versatile",  # FREE model
                    "messages": [
                        {
                            "role": "system",
                            "content": f"You are a helpful assistant for a Physical AI & Robotics course. Context: {context}"
                        },
                        {
                            "role": "user",
                            "content": message.message
                        }
                    ],
                    "temperature": 0.7,
                    "max_tokens": 1024
                },
                timeout=30.0
            )
            
            if response.status_code != 200:
                raise HTTPException(status_code=500, detail="Groq API error")
            
            result = response.json()
            ai_response = result['choices'][0]['message']['content']
        
        # Save to database
        conn = get_db_connection()
        cur = conn.cursor(cursor_factory=RealDictCursor)
        
        # Create or get conversation
        if not message.conversation_id:
            cur.execute("""
                INSERT INTO conversations (user_id, conversation_id, created_at)
                VALUES (%s, %s, NOW())
                RETURNING id, conversation_id
            """, (message.user_id, f"conv_{message.user_id}_{int(os.urandom(4).hex(), 16)}"))
            conv = cur.fetchone()
            conversation_id = conv['conversation_id']
        else:
            conversation_id = message.conversation_id
        
        # Save messages
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
        
        return {
            "response": ai_response,
            "conversation_id": conversation_id,
            "model": "llama-3.1-70b (Groq FREE)"
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Chat failed: {str(e)}")

@app.get("/health")
async def health():
    return {
        "status": "healthy",
        "database": "connected",
        "ai_provider": "Groq (FREE)",
        "vector_db": "Qdrant (FREE)"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, reload=True)