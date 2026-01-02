#!/bin/bash
# Script to run the backend server for development

echo "Starting the backend server for development..."
cd auth-backend
python -m uvicorn main:app --reload --port 8000