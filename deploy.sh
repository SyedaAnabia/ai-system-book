#!/bin/bash
# Deployment script for ai-system-book

echo "AI System Book Deployment Script"
echo "================================="

if [ "$1" = "github" ]; then
    echo "Building for GitHub Pages..."
    npm run build:github
    echo "GitHub Pages build completed. Deploy using: npm run gh-deploy"
elif [ "$1" = "vercel" ]; then
    echo "Building for Vercel..."
    npm run build:vercel
    echo "Vercel build completed. Deploy using: vercel --prod"
else
    echo "Usage: ./deploy.sh [github|vercel]"
    echo ""
    echo "github - Build for GitHub Pages deployment"
    echo "vercel - Build for Vercel deployment"
fi