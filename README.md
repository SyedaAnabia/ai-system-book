# Physical AI & Humanoid Robotics

Welcome to the **Physical AI & Humanoid Robotics** textbook repository. This is an AI-native textbook that teaches how to bridge the gap between digital AI and physical robotics.

## About This Project

This comprehensive textbook covers building intelligent robots that operate in the physical world. The content is structured around four core modules covering the complete Physical AI stack—from the robotic nervous system (ROS 2) to vision-language-action models that enable natural human-robot interaction.

### Modules Include:
- **Module 1**: ROS 2 - The Robotic Nervous System
- **Module 2**: Robot Kinematics and Dynamics - The Robotic Musculoskeletal System
- **Module 3**: The AI-Robot Brain
- **Module 4**: Vision-Language-Action (VLA)

## Live Website

The textbook is published at: [https://SyedaAnabia.github.io/ai-system-book/](https://SyedaAnabia.github.io/ai-system-book/)

## Deployment Options

### GitHub Pages (Default)
To deploy to GitHub Pages:
1. Push your changes to the main branch
2. Run the deployment command:
   ```bash
   npm run gh-deploy
   ```
3. Enable GitHub Pages in your repository settings, selecting the `gh-pages` branch

### Vercel
To deploy to Vercel:
1. Install the Vercel CLI: `npm i -g vercel`
2. Build for Vercel: `npm run build:vercel`
3. Deploy: `vercel --prod` (or connect your GitHub repo to Vercel dashboard)

### Backend Deployment (Required for Chatbot)

To deploy the backend API that powers the chatbot:

#### Option 1: Deploy to Render (Recommended)
1. Sign up at [https://render.com](https://render.com)
2. Create a new Web Service
3. Connect to your GitHub repository
4. Use the `render.yaml` file in this repository
5. Set the required environment variables in Render dashboard:
   - `DATABASE_URL`
   - `GROQ_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
6. Update the `API_BASE_URL` in `src/components/HomepageFeatures/FloatingChatbot.tsx` with your Render URL

#### Option 2: Deploy to Railway
1. Sign up at [https://railway.app](https://railway.app)
2. Import your repository
3. Set the required environment variables
4. Deploy the project
5. Update the `API_BASE_URL` in `src/components/HomepageFeatures/FloatingChatbot.tsx` with your Railway URL

#### Option 3: Deploy to Heroku
1. Sign up at [https://heroku.com](https://heroku.com)
2. Install Heroku CLI
3. Create a new app
4. Set the required environment variables
5. Deploy using Git
6. Update the `API_BASE_URL` in `src/components/HomepageFeatures/FloatingChatbot.tsx` with your Heroku URL

## Local Development

To run this textbook locally:

1. Clone the repository:
   ```bash
   git clone https://github.com/SyedaAnabia/ai-system-book.git
   cd ai-system-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm run start
   ```

4. Open your browser to [http://localhost:3000](http://localhost:3000) to view the textbook.

## Technologies Used

- [Docusaurus](https://docusaurus.io/): Static site generator optimized for documentation
- React: Component-based UI library
- TypeScript: Type-safe JavaScript
- GitHub Pages / Vercel: Hosting platforms

## Contributing

To contribute to this textbook:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This textbook is open source and available under the MIT License.
