import os
import requests
import glob
from pathlib import Path
import re
import time
from typing import Dict, Optional

# Configuration
DOCS_FOLDER = r"C:\Users\ayana\Desktop\chatbott\ai-system-book\docs"
API_URL = "http://localhost:8000/api/documents/upload"

# Progress tracking
total_files = 0
uploaded_files = 0
failed_files = 0


def extract_chapter_info(file_path: str) -> str:
    """
    Extract chapter information from the markdown file.
    This function looks for the first H1 heading in the file to determine the chapter name.
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Look for the first H1 heading (either # Title or <h1>Title</h1>)
        h1_match = re.search(r'^#\s+(.+)$|<h1>(.+?)</h1>', content, re.MULTILINE | re.IGNORECASE)

        if h1_match:
            # Return the first non-None group (either from # Title or <h1>Title</h1>)
            chapter_name = h1_match.groups()[0] or h1_match.groups()[1]
            return chapter_name.strip()
        else:
            # If no H1 heading found, use the filename as the chapter name
            return Path(file_path).stem.replace('-', ' ').replace('_', ' ').title()
    except Exception:
        # If there's an error, return the filename as the chapter name
        return Path(file_path).stem.replace('-', ' ').replace('_', ' ').title()


def extract_content(file_path: str) -> str:
    """
    Extract the content from the markdown file, excluding frontmatter if present.
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Remove frontmatter if present (content between --- delimiters at the beginning)
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) >= 3:
            content = parts[2]  # Skip the frontmatter and first ---

    return content.strip()


def upload_document(text: str, metadata: Dict) -> Optional[Dict]:
    """
    Upload a document to the API endpoint.
    """
    try:
        response = requests.post(
            API_URL,
            json={
                "text": text,
                "metadata": metadata
            },
            headers={"Content-Type": "application/json"},
            timeout=60
        )

        if response.status_code == 200:
            return response.json()
        else:
            print(f"❌ Upload failed with status {response.status_code}: {response.text}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"❌ Request error: {str(e)}")
        return None


def process_markdown_file(file_path: str):
    """
    Process a single markdown file: extract content and metadata, then upload.
    """
    global uploaded_files, failed_files

    try:
        # Extract content and metadata
        content = extract_content(file_path)
        chapter_info = extract_chapter_info(file_path)
        filename = Path(file_path).name

        # Prepare metadata
        metadata = {
            "source": filename,
            "chapter": chapter_info
        }

        # Upload to API
        result = upload_document(content, metadata)

        if result:
            uploaded_files += 1
            print(f"✅ [{uploaded_files}/{total_files}] Uploaded: {filename} (Chapter: {chapter_info})")
            return True
        else:
            failed_files += 1
            print(f"❌ [{failed_files}] Failed to upload: {filename}")
            return False

    except Exception as e:
        failed_files += 1
        print(f"❌ [{failed_files}] Error processing {Path(file_path).name}: {str(e)}")
        return False


def get_all_markdown_files(docs_folder: str) -> list:
    """
    Get all markdown files from the docs folder and its subdirectories.
    """
    markdown_files = []

    # Look for .md and .mdx files in the docs folder and subfolders
    for ext in ['*.md', '*.mdx']:
        pattern = os.path.join(docs_folder, '**', ext)
        markdown_files.extend(glob.glob(pattern, recursive=True))

    return sorted(markdown_files)


def main():
    global total_files, uploaded_files, failed_files

    print("📚 Starting upload of course book data to Qdrant vector database...")
    print(f"📂 Reading from: {DOCS_FOLDER}")
    print(f"🔗 API endpoint: {API_URL}")
    print("⚠️  Make sure your FastAPI backend is running on http://localhost:8000")
    print("-" * 60)

    # Check if API is accessible
    try:
        response = requests.get("http://localhost:8000/health", timeout=10)
        if response.status_code == 200:
            print("✅ Backend API is accessible")
        else:
            print("❌ Backend API is not responding. Please start your FastAPI server.")
            return
    except requests.exceptions.RequestException:
        print("❌ Could not connect to backend API. Please start your FastAPI server.")
        return

    # Get all markdown files
    markdown_files = get_all_markdown_files(DOCS_FOLDER)
    total_files = len(markdown_files)

    if total_files == 0:
        print("❌ No markdown files found in the docs folder!")
        return

    print(f"📋 Found {total_files} markdown files to upload")
    print()

    # Start timer
    start_time = time.time()

    # Process each file
    for i, file_path in enumerate(markdown_files, 1):
        print(f"⏳ Processing ({i}/{total_files}): {Path(file_path).name}")
        process_markdown_file(file_path)
        print()  # Empty line for readability

    # Calculate elapsed time
    elapsed_time = time.time() - start_time

    # Print summary
    print("=" * 60)
    print("📊 UPLOAD SUMMARY")
    print(f"   Total files processed: {total_files}")
    print(f"   Successfully uploaded: {uploaded_files}")
    print(f"   Failed uploads: {failed_files}")
    print(f"   Time elapsed: {elapsed_time:.2f} seconds")
    print("=" * 60)

    if failed_files > 0:
        print(f"\n⚠️  {failed_files} files failed to upload. Please check the errors above.")
    else:
        print(f"\n🎉 All {uploaded_files} files uploaded successfully!")
        print(f"Your chatbot should now be able to answer questions based on your book content!")


if __name__ == "__main__":
    main()