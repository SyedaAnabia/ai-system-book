import psycopg2
import bcrypt
import os
from dotenv import load_dotenv
import json

load_dotenv()
DATABASE_URL = os.getenv("DATABASE_URL")

conn = psycopg2.connect(DATABASE_URL)
cur = conn.cursor()

# Check if user exists
cur.execute("SELECT id FROM users WHERE email = %s", ('test@example.com',))
if cur.fetchone():
    print("✅ Test user already exists!")
else:
    # Create test user with id=1
    hashed_password = bcrypt.hashpw('password123'.encode('utf-8'), bcrypt.gensalt())
    
    cur.execute("""
        INSERT INTO users (
            id, name, email, password, software_experience, 
            programming_languages, has_gpu, gpu_type, 
            ros_experience, robotics_projects, learning_goals, 
            hardware_access, created_at
        )
        VALUES (1, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, NOW())
        ON CONFLICT (id) DO NOTHING
    """, (
        'Test User',
        'test@example.com',
        hashed_password.decode('utf-8'),
        'intermediate',
        json.dumps(['Python', 'JavaScript']),
        True,
        'RTX 3080',
        'beginner',
        True,
        'Learn Physical AI',
        json.dumps(['Computer', 'GPU'])
    ))
    
    conn.commit()
    print("✅ Test user created successfully!")

cur.close()
conn.close()