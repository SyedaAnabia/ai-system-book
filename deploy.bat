@echo off
REM Deployment script for ai-system-book

echo AI System Book Deployment Script
echo ================================

if "%1"=="github" (
    echo Building for GitHub Pages...
    npm run build:github
    echo GitHub Pages build completed. Deploy using: npm run gh-deploy
) else if "%1"=="vercel" (
    echo Building for Vercel...
    npm run build:vercel
    echo Vercel build completed. Deploy using: vercel --prod
) else (
    echo Usage: deploy.bat [github^|vercel]
    echo.
    echo github - Build for GitHub Pages deployment
    echo vercel - Build for Vercel deployment
)