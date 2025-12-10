import React, { useState, useRef, useEffect } from 'react';

interface Message {
  text: string;
  isUser: boolean;
  timestamp: Date;
}

const FloatingChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [hasGreeted, setHasGreeted] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    if (isOpen && !hasGreeted) {
      setMessages([{
        text: "Hello! 👋 Welcome to the Physical AI & Humanoid Robotics course. I'm your AI assistant. How can I help you today?",
        isUser: false,
        timestamp: new Date()
      }]);
      setHasGreeted(true);
    }
  }, [isOpen, hasGreeted]);

  const getSmartResponse = (userMessage: string): string => {
    const msg = userMessage.toLowerCase().trim();

    // Simple greetings
    if (msg === 'hi' || msg === 'hello' || msg === 'hey' || msg === 'hi!' || msg === 'hello!') {
      return "Hi! 👋 How can I assist you today?";
    }

    if (msg.includes('how are you') || msg.includes('what\'s up') || msg.includes('whats up')) {
      return "I'm doing great, thank you for asking! How can I help you with the course today?";
    }

    // Course/Module questions
    if (msg.includes('module 1') || msg.includes('module one') || msg.match(/\bmodule\s*1\b/)) {
      return "📘 **Module 1: Introduction to Physical AI & Humanoid Robotics**\n\n" +
        "- Overview of Physical AI: what it is and where it's used.\n" +
        "- Humanoid robot architecture: sensors, actuators, controllers.\n" +
        "- Basic electronics and embedded components.\n" +
        "- Safety, ethics, and system-level thinking.\n\n" +
        "Would you like more details about Module 1?";
    }

    if (msg.includes('module 2') || msg.includes('module two') || msg.match(/\bmodule\s*2\b/)) {
      return "📘 **Module 2: ROS (Robot Operating System) Fundamentals**\n\n" +
        "- ROS architecture: nodes, topics, services and actions.\n" +
        "- Writing & running simple ROS nodes.\n" +
        "- Interfacing sensors and actuators.\n" +
        "- Simulation with Gazebo and visualizing in RViz.\n\n" +
        "Module 2 includes practical examples and starter templates!";
    }

    if (msg.includes('module 3') || msg.includes('module three') || msg.match(/\bmodule\s*3\b/)) {
      return "📘 **Module 3: Computer Vision & Perception**\n\n" +
        "- Camera basics, calibration, and image preprocessing.\n" +
        "- Object detection & recognition (classical + ML methods).\n" +
        "- Using OpenCV and integrating pretrained models.\n" +
        "- Sensor fusion (camera + IMU / LIDAR) basics.";
    }

    if (msg.includes('module 4') || msg.includes('module four') || msg.match(/\bmodule\s*4\b/)) {
      return "📘 **Module 4: Motion Planning & Control**\n\n" +
        "- Forward/inverse kinematics and robot dynamics overview.\n" +
        "- Path planning algorithms (A*, RRT, PRM) and their tradeoffs.\n" +
        "- Control systems: PID, state-feedback, and basic trajectory tracking.\n" +
        "- Sim-to-real tips using Gazebo/Unity.";
    }

    if (msg.includes('module 5') || msg.includes('module five') || msg.match(/\bmodule\s*5\b/)) {
      return "📘 **Module 5: Deep Learning for Robotics**\n\n" +
        "- Neural networks basics and architectures useful for robotics.\n" +
        "- Using CNNs for perception, RNNs for sequences, and RL basics for control.\n" +
        "- Transfer learning and model deployment on edge devices.";
    }

    if (msg.includes('module 6') || msg.includes('module six') || msg.match(/\bmodule\s*6\b/)) {
      return "📘 **Module 6: Building Your First Humanoid**\n\n" +
        "- Mechanical design choices and trade-offs.\n" +
        "- Electronics: power, motor drivers, and sensors.\n" +
        "- Software integration: combining perception, planning, control.\n" +
        "- Testing, debugging and iteration cycle for humanoid builds.";
    }

    if (msg.includes('modules') || msg.includes('syllabus') || msg.includes('curriculum') || msg.includes('all modules')) {
      return "📚 **Course Modules Overview:**\n\n" +
        "**Module 1:** Introduction to Physical AI & Humanoid Robotics\n" +
        "**Module 2:** ROS (Robot Operating System) Fundamentals\n" +
        "**Module 3:** Computer Vision & Perception\n" +
        "**Module 4:** Motion Planning & Control\n" +
        "**Module 5:** Deep Learning for Robotics\n" +
        "**Module 6:** Building Your First Humanoid\n\n" +
        "Which module would you like to know more about?";
    }

    // Website-related questions
    if (msg.includes('website') || msg.includes('site')) {
      return "🌐 **About Our Website:**\n\n" +
        "Our website offers:\n" +
        "- Complete course curriculum and modules\n" +
        "- Interactive learning materials\n" +
        "- Registration for the Physical AI & Humanoid Robotics course\n" +
        "- Resources and documentation\n" +
        "- Community support\n\n" +
        "What specific information about the website would you like to know?";
    }

    if (msg.includes('register') || msg.includes('registration') || msg.includes('sign up') || msg.includes('enroll')) {
      return "📝 **Course Registration:**\n\n" +
        "To register for the Physical AI & Humanoid Robotics course:\n" +
        "1. Click on the 'Register' or 'Sign Up' button on the homepage\n" +
        "2. Fill in your details (name, email, etc.)\n" +
        "3. Choose your preferred payment plan\n" +
        "4. Complete the registration process\n\n" +
        "Once registered, you'll get immediate access to all course materials!";
    }

    if (msg.includes('login') || msg.includes('log in') || msg.includes('signin') || msg.includes('sign in')) {
      return "🔐 **Login Information:**\n\n" +
        "To access your account:\n" +
        "1. Click the 'Login' button on the homepage\n" +
        "2. Enter your registered email and password\n" +
        "3. Access your course dashboard\n\n" +
        "Forgot your password? Use the 'Reset Password' link on the login page.";
    }

    if (msg.includes('price') || msg.includes('cost') || msg.includes('fee') || msg.includes('payment')) {
      return "💰 **Course Pricing:**\n\n" +
        "We offer flexible pricing options for the Physical AI & Humanoid Robotics course:\n" +
        "- One-time payment: Full course access\n" +
        "- Monthly installments: Split payment option\n" +
        "- Early bird discounts: Available for limited time\n\n" +
        "Visit our pricing page for current rates and special offers!";
    }

    if (msg.includes('contact') || msg.includes('support') || msg.includes('help desk')) {
      return "📞 **Contact & Support:**\n\n" +
        "Need assistance? Reach us at:\n" +
        "- Email: support@physicalai-course.com\n" +
        "- Live Chat: Available on website\n" +
        "- Support Hours: Mon-Fri, 9 AM - 6 PM\n\n" +
        "You can also use this chatbot for instant answers!";
    }

    if (msg.includes('certificate') || msg.includes('certification')) {
      return "🎓 **Course Certificate:**\n\n" +
        "Upon successful completion of the Physical AI & Humanoid Robotics course, you will receive:\n" +
        "- Official course completion certificate\n" +
        "- Verifiable credentials\n" +
        "- LinkedIn-ready certificate\n\n" +
        "Complete all 6 modules and pass the final project to earn your certificate!";
    }

    if (msg.includes('duration') || msg.includes('how long') || msg.includes('time required')) {
      return "⏱️ **Course Duration:**\n\n" +
        "The Physical AI & Humanoid Robotics course is self-paced:\n" +
        "- Estimated time: 8-12 weeks (at 10-15 hours/week)\n" +
        "- Flexible schedule: Learn at your own pace\n" +
        "- Lifetime access: Revisit materials anytime\n\n" +
        "You can complete it faster or slower based on your schedule!";
    }

    if (msg.includes('prerequisite') || msg.includes('requirements') || msg.includes('beginner')) {
      return "📋 **Course Prerequisites:**\n\n" +
        "Recommended background:\n" +
        "- Basic programming knowledge (Python preferred)\n" +
        "- Understanding of basic math and physics\n" +
        "- Enthusiasm for robotics!\n\n" +
        "Don't worry if you're a beginner - we start from the fundamentals and build up gradually!";
    }

    if (msg.includes('instructor') || msg.includes('teacher') || msg.includes('who teaches')) {
      return "👨‍🏫 **Course Instructors:**\n\n" +
        "Learn from industry experts with:\n" +
        "- Years of experience in robotics and AI\n" +
        "- Real-world project expertise\n" +
        "- Academic and industry backgrounds\n\n" +
        "Our instructors are passionate about making robotics accessible to everyone!";
    }

    if (msg.includes('project') || msg.includes('hands-on') || msg.includes('practical')) {
      return "🛠️ **Hands-on Projects:**\n\n" +
        "The course includes multiple practical projects:\n" +
        "- Module-specific mini-projects\n" +
        "- Final capstone project: Build your own humanoid robot\n" +
        "- Simulations and real-world implementations\n" +
        "- Code examples and starter templates\n\n" +
        "Learning by doing is at the core of our curriculum!";
    }

    if (msg.includes('video') || msg.includes('lectures') || msg.includes('tutorial')) {
      return "🎥 **Course Content Format:**\n\n" +
        "Our course includes:\n" +
        "- Video lectures and tutorials\n" +
        "- Written documentation and guides\n" +
        "- Code repositories and examples\n" +
        "- Interactive exercises\n" +
        "- Downloadable resources\n\n" +
        "Multiple formats to suit different learning styles!";
    }

    if (msg.includes('community') || msg.includes('forum') || msg.includes('discussion')) {
      return "👥 **Community & Support:**\n\n" +
        "Join our vibrant learning community:\n" +
        "- Student discussion forums\n" +
        "- Peer collaboration opportunities\n" +
        "- Regular Q&A sessions\n" +
        "- Project showcases\n\n" +
        "Connect with fellow students and learn together!";
    }

    // Default response for unmatched queries
    return "I'm here to help you with:\n\n" +
      "📚 **Course Information:** Modules, curriculum, content\n" +
      "🌐 **Website:** Registration, login, pricing\n" +
      "🎓 **Learning:** Prerequisites, duration, certificates\n" +
      "💬 **Support:** Contact info, community, help\n\n" +
      "What would you like to know more about?";
  };

  const handleSendMessage = () => {
    if (!inputValue.trim()) return;

    const userMessage: Message = {
      text: inputValue,
      isUser: true,
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    const messageText = inputValue;
    setInputValue('');
    setIsLoading(true);

    setTimeout(() => {
      const response = getSmartResponse(messageText);
      const botMessage: Message = {
        text: response,
        isUser: false,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
      setIsLoading(false);
    }, 1000);
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <>
      <style>{`
        .chat-toggle-btn {
          position: fixed;
          bottom: 24px;
          right: 24px;
          width: 60px;
          height: 60px;
          border-radius: 50%;
          background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
          border: none;
          color: white;
          cursor: pointer;
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
          display: flex;
          align-items: center;
          justify-content: center;
          transition: all 0.3s ease;
          z-index: 1000;
        }

        .chat-toggle-btn:hover {
          transform: scale(1.1);
          box-shadow: 0 6px 20px rgba(0, 0, 0, 0.25);
        }

        .chat-toggle-btn.active {
          background: linear-gradient(135deg, #764ba2 0%, #667eea 100%);
        }

        .chat-window {
          position: fixed;
          bottom: 100px;
          right: 24px;
          width: 380px;
          height: 600px;
          background: white;
          border-radius: 16px;
          box-shadow: 0 8px 32px rgba(0, 0, 0, 0.12);
          display: flex;
          flex-direction: column;
          opacity: 0;
          transform: translateY(20px) scale(0.95);
          pointer-events: none;
          transition: all 0.3s ease;
          z-index: 999;
          overflow: hidden;
        }

        .chat-window.open {
          opacity: 1;
          transform: translateY(0) scale(1);
          pointer-events: all;
        }

        .chat-header {
          background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
          color: white;
          padding: 20px;
          display: flex;
          justify-content: space-between;
          align-items: center;
        }

        .chat-header-info {
          display: flex;
          align-items: center;
          gap: 8px;
        }

        .chat-header h3 {
          margin: 0;
          font-size: 18px;
          font-weight: 600;
        }

        .status-indicator {
          width: 8px;
          height: 8px;
          background: #4ade80;
          borderRadius: 50%;
          animation: pulse 2s infinite;
        }

        .status-text {
          font-size: 12px;
          opacity: 0.9;
        }

        @keyframes pulse {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.5; }
        }

        .close-btn {
          background: rgba(255, 255, 255, 0.2);
          border: none;
          color: white;
          width: 32px;
          height: 32px;
          border-radius: 8px;
          cursor: pointer;
          font-size: 24px;
          display: flex;
          align-items: center;
          justify-content: center;
          transition: background 0.2s;
          line-height: 1;
        }

        .close-btn:hover {
          background: rgba(255, 255, 255, 0.3);
        }

        .chat-messages {
          flex: 1;
          overflow-y: auto;
          padding: 20px;
          background: #f8f9fa;
          display: flex;
          flex-direction: column;
          gap: 12px;
        }

        .chat-messages::-webkit-scrollbar {
          width: 6px;
        }

        .chat-messages::-webkit-scrollbar-track {
          background: transparent;
        }

        .chat-messages::-webkit-scrollbar-thumb {
          background: #cbd5e0;
          border-radius: 3px;
        }

        .message {
          display: flex;
          flex-direction: column;
          max-width: 75%;
          animation: slideIn 0.3s ease;
        }

        @keyframes slideIn {
          from {
            opacity: 0;
            transform: translateY(10px);
          }
          to {
            opacity: 1;
            transform: translateY(0);
          }
        }

        .user-message {
          align-self: flex-end;
        }

        .bot-message {
          align-self: flex-start;
        }

        .message-content {
          padding: 12px 16px;
          border-radius: 12px;
          word-wrap: break-word;
          white-space: pre-line;
          line-height: 1.5;
          font-size: 14px;
        }

        .user-message .message-content {
          background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
          color: white;
          border-radius: 12px 12px 4px 12px;
        }

        .bot-message .message-content {
          background: white;
          color: #1a202c;
          border: 1px solid #e2e8f0;
          border-radius: 12px 12px 12px 4px;
          box-shadow: 0 1px 2px rgba(0, 0, 0, 0.05);
        }

        .message-timestamp {
          font-size: 11px;
          color: #718096;
          margin-top: 4px;
          padding: 0 4px;
        }

        .user-message .message-timestamp {
          text-align: right;
        }

        .typing-indicator {
          display: flex;
          gap: 4px;
          padding: 16px !important;
        }

        .typing-indicator span {
          width: 8px;
          height: 8px;
          background: #667eea;
          border-radius: 50%;
          animation: typing 1.4s infinite;
        }

        .typing-indicator span:nth-child(2) {
          animation-delay: 0.2s;
        }

        .typing-indicator span:nth-child(3) {
          animation-delay: 0.4s;
        }

        @keyframes typing {
          0%, 60%, 100% {
            transform: translateY(0);
            opacity: 0.7;
          }
          30% {
            transform: translateY(-10px);
            opacity: 1;
          }
        }

        .chat-input-container {
          padding: 16px;
          background: white;
          border-top: 1px solid #e2e8f0;
          display: flex;
          gap: 12px;
          align-items: flex-end;
        }

        .chat-input {
          flex: 1;
          border: 1px solid #e2e8f0;
          border-radius: 12px;
          padding: 12px 16px;
          font-size: 14px;
          resize: none;
          font-family: inherit;
          outline: none;
          transition: border-color 0.2s;
          max-height: 100px;
          overflow-y: auto;
        }

        .chat-input:focus {
          border-color: #667eea;
        }

        .send-btn {
          width: 44px;
          height: 44px;
          background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
          border: none;
          border-radius: 12px;
          color: white;
          cursor: pointer;
          display: flex;
          align-items: center;
          justify-content: center;
          transition: all 0.2s;
          flex-shrink: 0;
        }

        .send-btn:hover:not(:disabled) {
          transform: scale(1.05);
          box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
        }

        .send-btn:disabled {
          opacity: 0.5;
          cursor: not-allowed;
        }

        @media (max-width: 480px) {
          .chat-window {
            width: calc(100vw - 32px);
            height: calc(100vh - 140px);
            right: 16px;
            bottom: 90px;
          }

          .chat-toggle-btn {
            right: 16px;
            bottom: 16px;
          }
        }
      `}</style>

      <button
        className={`chat-toggle-btn ${isOpen ? 'active' : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        )}
      </button>

      <div className={`chat-window ${isOpen ? 'open' : ''}`}>
        <div className="chat-header">
          <div className="chat-header-info">
            <h3>AI Assistant</h3>
            <span className="status-indicator"></span>
            <span className="status-text">Online</span>
          </div>
          <button
            className="close-btn"
            onClick={() => setIsOpen(false)}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>

        <div className="chat-messages">
          {messages.map((message, index) => (
            <div
              key={index}
              className={`message ${message.isUser ? 'user-message' : 'bot-message'}`}
            >
              <div className="message-content">
                {message.text}
              </div>
              <div className="message-timestamp">
                {message.timestamp.toLocaleTimeString([], { 
                  hour: '2-digit', 
                  minute: '2-digit' 
                })}
              </div>
            </div>
          ))}
          {isLoading && (
            <div className="message bot-message">
              <div className="message-content typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <div className="chat-input-container">
          <input
            type="text"
            className="chat-input"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="Type your message..."
          />
          <button
            className="send-btn"
            onClick={handleSendMessage}
            disabled={!inputValue.trim() || isLoading}
            aria-label="Send message"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="22" y1="2" x2="11" y2="13"></line>
              <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
            </svg>
          </button>
        </div>
      </div>
    </>
  );
};

export default FloatingChatbot;