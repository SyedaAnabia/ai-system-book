module.exports = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 - The Robotic Nervous System',
      items: [
        'module1/chapter1-intro-ros2-architecture',
        'module1/chapter2-nodes-topics-services',
        'module1/chapter3-services-actions',
        'module1/chapter4-params-launch',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Robot Kinematics and Dynamics - The Robotic Musculoskeletal System',
      items: [
        'module2/chapter1-overview',
        'module2/chapter2-gazebo-physics-simulation',
        'module2/chapter3-unity-environment-building',
        'module2/chapter4-sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      items: [
        'module3/chapter1-overview',
        'module3/chapter2-nvidia-isaac-sim',
        'module3/chapter3-isaac-ros-vslam',
        'module3/chapter4-nav2-bipedal-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/chapter1-overview',
        'module4/chapter2-voice-to-action',
        'module4/chapter3-cognitive-planning',
        'module4/chapter4-capstone-autonomous-humanoid',
      ],
    },
  ],
};
