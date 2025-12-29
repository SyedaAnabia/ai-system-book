import os
import httpx
import asyncio
from pathlib import Path
import time

import os
from dotenv import load_dotenv

load_dotenv()

# Your API endpoint
API_URL = os.getenv("API_URL", "http://localhost:8000/api/documents/upload")

# Progress tracking
total_files = 0
uploaded_files = 0
failed_files = 0

async def upload_markdown_file(file_path: str):
    """Upload a single markdown file"""
    global uploaded_files, failed_files

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Skip empty files
        if len(content.strip()) < 100:
            print(f"⏭️  Skipped (too short): {Path(file_path).name}")
            return

        # Extract metadata from file
        file_name = Path(file_path).name

        async with httpx.AsyncClient() as client:
            response = await client.post(
                API_URL,
                json={
                    "text": content,
                    "metadata": {
                        "source": file_name,
                        "file_path": file_path
                    }
                },
                timeout=60.0
            )

            if response.status_code == 200:
                result = response.json()
                uploaded_files += 1
                print(f"✅ [{uploaded_files}/{total_files}] {file_name} ({result['chunks_created']} chunks)")
            else:
                failed_files += 1
                print(f"❌ Failed: {file_name} - {response.text}")

    except Exception as e:
        failed_files += 1
        print(f"❌ Error: {Path(file_path).name} - {str(e)}")

async def upload_directory(directory: str):
    """Upload all markdown files from a directory"""
    global total_files

    files_to_upload = []

    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(('.md', '.mdx')):
                file_path = os.path.join(root, file)
                files_to_upload.append(file_path)

    total_files = len(files_to_upload)

    if total_files == 0:
        print(f"❌ No .md or .mdx files found in {directory}")
        return

    print(f"\n📚 Found {total_files} markdown files")
    print(f"📂 Directory: {directory}")
    print(f"🚀 Starting upload...\n")

    start_time = time.time()

    # Upload with concurrency limit
    tasks = []
    for file_path in files_to_upload:
        tasks.append(upload_markdown_file(file_path))

        # Upload in batches of 5
        if len(tasks) == 5:
            await asyncio.gather(*tasks)
            tasks = []
            await asyncio.sleep(0.5)  # Small delay between batches

    # Upload remaining
    if tasks:
        await asyncio.gather(*tasks)

    elapsed_time = time.time() - start_time

    print(f"\n{'='*60}")
    print(f"✅ Upload Complete!")
    print(f"   Total files: {total_files}")
    print(f"   Successful: {uploaded_files}")
    print(f"   Failed: {failed_files}")
    print(f"   Time taken: {elapsed_time:.2f}s")
    print(f"{'='*60}\n")

if __name__ == "__main__":
    # Replace with your Docusaurus docs folder path
    DOCS_FOLDER = input("Enter path to your docs folder (e.g., ./docs or ../docs): ").strip()

    if not os.path.exists(DOCS_FOLDER):
        print(f"❌ Error: Directory '{DOCS_FOLDER}' not found!")
        exit(1)

    asyncio.run(upload_directory(DOCS_FOLDER))