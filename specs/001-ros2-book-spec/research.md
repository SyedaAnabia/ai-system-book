# Research: AI Systems in the Physical World - Embodied Intelligence

**Feature**: 001-ros2-book-spec
**Date**: 2025-12-09

This research document addresses all unknowns identified in the technical context and provides the information needed to resolve all "NEEDS CLARIFICATION" markers.

## Technology Decisions & Rationale

### Decision: Docusaurus as Documentation Framework
**Rationale**: Docusaurus is the ideal choice for this technical book because:
- It's designed specifically for documentation sites with built-in features for technical content
- Offers excellent support for MDX (Markdown with React components) for interactive content
- Provides built-in search functionality that is essential for a technical reference
- Supports versioning which will be important for future book editions
- Has strong community support and extensive documentation
- Integrates well with GitHub Pages for free hosting

**Alternatives considered**:
- GitBook: More limited customization options and requires proprietary format
- Hugo: Requires more configuration and frontmatter knowledge from contributors
- Custom React app: More complex to maintain and lacks documentation-specific features

### Decision: GitHub Pages for Deployment
**Rationale**: GitHub Pages provides:
- Cost-free hosting with reliable uptime
- Seamless integration with version control workflow
- Support for custom domains
- Built-in CDN for global distribution
- Automatic deployment via GitHub Actions

**Alternatives considered**:
- Netlify: Requires additional setup outside GitHub workflow
- AWS S3/CloudFront: Incurs ongoing costs and requires more maintenance
- Self-hosted: Significantly more complex and costly

### Decision: Python 3.8+ for Code Examples
**Rationale**: 
- Python is the most common language for robotics and AI applications
- Most ROS 2 packages have Python APIs
- Easy for students to learn and understand
- Compatible with all major frameworks used in the book (ROS 2, Isaac, etc.)

**Alternatives considered**:
- C++: More complex for beginners, though important for performance-critical applications
- Both C++ and Python: Would double the maintenance burden without significant pedagogical benefit

### Decision: ROS 2 Iron/Iguana as Target Distribution
**Rationale**:
- ROS 2 Iron is the current LTS (Long Term Support) distribution
- Has the best support for all features covered in the book
- Will remain supported for the next 2+ years ensuring longevity
- Good compatibility with simulation environments and NVIDIA Isaac

**Alternatives considered**:
- Humble Hawksbill: Previous LTS, missing some newer features
- Rolling Ridley: Bleeding edge but unstable for book content

### Decision: Gazebo Garden for Simulation
**Rationale**:
- Gazebo Garden is the current stable version with OGRE 2.x rendering
- Better performance and rendering quality than older versions
- Good integration with ROS 2
- Industry standard for robotics simulation

**Alternatives considered**:
- Gazebo Classic: Legacy version, no longer actively developed
- Ignition Gazebo: Previous generation (has been renamed to Gazebo Garden)

## Best Practices & Integration Patterns

### Documentation Best Practices
1. Each chapter follows the same structure: learning objectives, content, practical examples, exercises, summary
2. Code examples are complete and runnable, not just snippets
3. Each example includes expected output or behavior
4. Visual aids are high-resolution and clearly labeled
5. Cross-module references are clearly marked to help students connect concepts

### Content Development Workflow
1. Write chapter content with learning objectives
2. Develop and test all code examples in actual environments
3. Create visual aids to support concepts
4. Develop practical exercises with clear instructions
5. Conduct technical review with domain expert
6. Conduct pedagogical review for clarity and effectiveness

### Code Example Standards
1. Examples must work in the described environment without modification
2. Examples should be educational, not overly complex
3. Include appropriate error handling and comments
4. Follow ROS 2 and Python best practices
5. Include setup instructions for each example

## Architecture & Design Decisions

### Progressive Learning Structure
Based on the book's constitution principle of "Progressive Complexity", the modules are structured as follows:

1. **Module 1 (ROS 2)**: Foundation layer - introduces core concepts of robotic systems
2. **Module 2 (Gazebo/Unity)**: Simulation layer - builds on ROS 2 with virtual environments
3. **Module 3 (NVIDIA Isaac)**: Advanced platform layer - integrates hardware acceleration
4. **Module 4 (Vision-Language-Action)**: Application layer - combines perception, reasoning, and action

This structure ensures that each module builds on the previous ones, following the "Progressive Complexity" principle from the constitution.

### Asset Organization Strategy
To maintain consistency and ease of maintenance across the book's multiple modules:

1. Module-specific directories for all content and code examples
2. Shared assets (images, diagrams) organized by concept area
3. Consistent naming conventions for code files and examples
4. Centralized documentation for environment setup and testing procedures
5. Standardized format for exercises and capstone project requirements