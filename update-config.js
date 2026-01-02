const fs = require('fs');
const path = require('path');

// Determine the target platform based on environment variable
const platform = process.env.DEPLOY_PLATFORM || 'github'; // Default to GitHub Pages

// Read the original config
let configContent = fs.readFileSync('./docusaurus.config.ts', 'utf8');

// Modify the config based on the platform
if (platform === 'vercel') {
  // For Vercel deployment
  configContent = configContent.replace(
    "url: 'https://SyedaAnabia.github.io',",
    "url: 'https://ai-system-book-gi5z.vercel.app', // Vercel deployment URL"
  );
  configContent = configContent.replace(
    "baseUrl: '/ai-system-book/',",
    "baseUrl: '/',"
  );
  console.log('Config updated for Vercel deployment');
} else {
  // For GitHub Pages (default)
  configContent = configContent.replace(
    "url: 'https://ai-system-book-gi5z.vercel.app', // Vercel deployment URL",
    "url: 'https://SyedaAnabia.github.io',"
  );
  configContent = configContent.replace(
    "baseUrl: '/',",
    "baseUrl: '/ai-system-book/',"
  );
  console.log('Config updated for GitHub Pages deployment');
}

// Write the updated config
fs.writeFileSync('./docusaurus.config.ts', configContent);
console.log('Docusaurus config updated successfully!');