# Feature Specification: Chapter 2 - Nodes, Topics, and Services

**Feature Branch**: `002-chapter2-nodes-topics-services`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Based on the constitution, create detailed specifications for Chapter 2: \"Nodes, Topics, and Services\" CHAPTER STRUCTURE: Section 1: UNDERSTANDING ROS 2 NODES - Subsection 1.1: What is a Node? (300-400 words) * Definition and purpose * Node lifecycle * When to create new nodes vs reusing existing ones - Subsection 1.2: Node Anatomy (400-500 words) * Node initialization * Node configuration * Node parameters * Code Example 1: Minimal node (10-15 lines) - Subsection 1.3: Creating Your First Node (600-800 words) * Step-by-step tutorial * Code Example 2: Complete node with logging (30-40 lines) * Running and testing the node * Common errors and solutions Section 2: TOPICS - PUBLISH/SUBSCRIBE PATTERN - Subsection 2.1: Introduction to Topics (300-400 words) * What are topics? * Publish-Subscribe pattern explained * Use cases and scenarios - Subsection 2.2: Message Types (400-500 words) * Standard ROS 2 message types * String, Int, Float, Custom messages * How to find and use message types * Code Example 3: Exploring message types (CLI commands) - Subsection 2.3: Creating a Publisher (700-900 words) * Publisher node anatomy * Code Example 4: String publisher (40-50 lines) * Publishing rate and timing * Quality of Service (QoS) basics * Testing with ros2 topic echo - Subsection 2.4: Creating a Subscriber (700-900 words) * Subscriber node anatomy * Callback functions explained * Code Example 5: String subscriber (40-50 lines) * Testing publisher-subscriber communication - Subsection 2.5: Practical Example - Temperature Monitor (1000-1200 words) * Real-world scenario * Code Example 6: Temperature publisher (60-80 lines) * Code Example 7: Temperature subscriber with alerts (60-80 lines) * Running multiple nodes * Visualization with rqt_graph Section 3: SERVICES - REQUEST/RESPONSE PATTERN - Subsection 3.1: Understanding Services (300-400 words) * What are services? * Request-Response pattern * Topics vs Services - when to use what? - Subsection 3.2: Service Types (400-500 words) * Standard service types * Service definition files (.srv) * Request and Response structure * Code Example 8: Examining service types - Subsection 3.3: Creating a Service Server (800-1000 words) * Service server anatomy * Code Example 9: AddTwoInts service server (50-70 lines) * Handling requests * Sending responses * Error handling - Subsection 3.4: Creating a Service Client (700-900 words) * Service client anatomy * Code Example 10: AddTwoInts service client (50-70 lines) * Synchronous vs Asynchronous calls * Timeout handling - Subsection 3.5: Practical Example - Robot Controller (1000-1200 words) * Real-world scenario * Code Example 11: Motion service server (80-100 lines) * Code Example 12: Motion service client (60-80 lines) * Integration testing Section 4: ADVANCED CONCEPTS - Subsection 4.1: Quality of Service (QoS) (600-800 words) * QoS profiles explained * Reliability, Durability, History * Code Example 13: Publisher with custom QoS (30-40 lines) * When to use different QoS settings - Subsection 4.2: Node Composition (500-700 words) * Component nodes * Benefits of composition * Code Example 14: Composable node (40-50 lines) - Subsection 4.3: Lifecycle Nodes (500-700 words) * Managed nodes * State transitions * Use cases for lifecycle management Section 5: DEBUGGING AND TOOLS - Subsection 5.1: ROS 2 CLI Tools (600-800 words) * ros2 node list/info * ros2 topic list/echo/pub * ros2 service list/call * Practical examples for each - Subsection 5.2: Visualization Tools (500-700 words) * rqt_graph for system visualization * rqt_console for logging * rqt_plot for data visualization Section 6: HANDS-ON PROJECT - Project: Multi-Node Robot Telemetry System (1500-2000 words) * Requirements specification * System architecture diagram * Code Example 15: Sensor node (100-120 lines) * Code Example 16: Data processor node (100-120 lines) * Code Example 17: Control service (80-100 lines) * Code Example 18: Dashboard subscriber (80-100 lines) * Integration and testing guide * Extension challenges Section 7: EXERCISES AND CHALLENGES - Exercise 1: Basic Node Creation (Easy) - Exercise 2: Publisher-Subscriber Chain (Medium) - Exercise 3: Service-Based Calculator (Medium) - Exercise 4: Multi-Node Communication System (Hard) - Exercise 5: Custom Message Types (Hard) - Challenge Project: Design your own system (Advanced) Section 8: SUMMARY AND NEXT STEPS - Key Takeaways (5-7 bullet points) - Concept Map / Mind Map - Preview of Chapter 3 - Additional Resources (10-15 links) - FAQ (8-10 common questions) TECHNICAL SPECIFICATIONS: - Total word count: 10,000-12,000 words - Code examples: 18 complete examples - Diagrams needed: 8-10 (architecture, flowcharts, sequence diagrams) - Estimated reading time: 2-3 hours - Hands-on practice time: 4-6 hours STYLE REQUIREMENTS: - Use clear, simple language - Explain jargon when first introduced - Include "💡 Pro Tip" callouts (5-7 throughout) - Include "⚠️ Common Pitfall" warnings (5-7 throughout) - Use analogies to explain complex concepts - Add "✅ Checkpoint" sections after major concepts Format this as a detailed outline with word counts and specifications for each section."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learns Node Fundamentals (Priority: P1)

As a student who has completed Chapter 1 (Introduction to ROS 2), I want to understand what nodes are, how to create them, and their lifecycle so that I can build the foundation for more complex robotic applications.

**Why this priority**: Nodes are the fundamental building blocks of ROS 2 systems, and understanding them is essential before moving to communication patterns. This must be mastered first to progress through the chapter.

**Independent Test**: Students can create a minimal ROS 2 node with logging, run it, and verify it appears in the ROS 2 system using ros2 node list command.

**Acceptance Scenarios**:

1. **Given** a student with basic Python knowledge and ROS 2 installed, **When** they complete Section 1, **Then** they can explain what a node is, its lifecycle, and when to create new nodes vs reusing existing ones.
2. **Given** the need to create a simple ROS 2 node, **When** the student implements it using rclpy, **Then** they create a working node with proper initialization and logging.
3. **Given** a node running in the system, **When** the student runs ROS 2 CLI commands, **Then** they can identify and inspect the node properties.

---

### User Story 2 - Student Implements Publish/Subscribe Communication (Priority: P2)

As a student who understands nodes, I want to learn about the publish/subscribe communication pattern so that I can create systems where information flows between different parts of my robotic application.

**Why this priority**: The pub/sub pattern is the most common communication method in ROS 2 and builds directly on node knowledge from Section 1.

**Independent Test**: Students can create a publisher node that sends messages and a subscriber node that receives them, with successful communication validated through ros2 topic echo.

**Acceptance Scenarios**:

1. **Given** a need for one-way data flow, **When** the student implements a publisher-subscriber system, **Then** messages flow reliably from publisher to subscriber.
2. **Given** the need to use standard message types, **When** the student implements them, **Then** they can correctly import and use ROS 2 message types.
3. **Given** the requirement to test communication, **When** the student runs both nodes, **Then** they can verify successful message exchange.

---

### User Story 3 - Student Creates Service-Based Interactions (Priority: P3)

As a student who knows about nodes and topics, I want to learn about services (request/response pattern) so that I can create synchronous communication between nodes when needed.

**Why this priority**: Services provide the other major communication pattern in ROS 2, essential for request-response interactions that topics can't handle effectively.

**Independent Test**: Students can create a service server and client that successfully handle requests and responses with proper error handling.

**Acceptance Scenarios**:

1. **Given** the need for synchronous request-response communication, **When** the student creates a service server and client, **Then** requests are properly handled and responses are returned.
2. **Given** potential service errors, **When** the student implements error handling, **Then** the system gracefully handles error conditions.
3. **Given** the need to distinguish when to use services vs topics, **When** the student evaluates scenarios, **Then** they correctly choose the appropriate communication pattern.

---

### User Story 4 - Student Masters Advanced Concepts (Priority: P4)

As a student who understands basic nodes and communication, I want to explore advanced concepts like QoS, node composition, and lifecycle management so that I can build production-ready robotic systems.

**Why this priority**: These concepts are important for creating robust and scalable ROS 2 applications, but they build on the basic communication patterns.

**Independent Test**: Students can implement node composition and configure QoS settings appropriately for different use cases.

**Acceptance Scenarios**:

1. **Given** quality of service requirements, **When** the student configures QoS profiles, **Then** the communication behaves according to the chosen reliability and durability settings.
2. **Given** the need for efficient node management, **When** the student uses node composition, **Then** components run within a single process with shared resources.
3. **Given** the need for lifecycle management, **When** the student implements lifecycle nodes, **Then** they can manage node state transitions effectively.

---

### User Story 5 - Student Completes Hands-On Project (Priority: P5)

As a student who has learned all concepts in the chapter, I want to complete a comprehensive multi-node project that combines everything learned so that I can demonstrate my understanding of ROS 2 communication patterns.

**Why this priority**: The hands-on project provides an opportunity to integrate all learning from the chapter and validate understanding through practical application.

**Independent Test**: Students can build, run, and test a complete multi-node robotic telemetry system that successfully integrates nodes, topics, and services.

**Acceptance Scenarios**:

1. **Given** project requirements, **When** the student implements the complete system, **Then** all components communicate effectively using appropriate patterns.
2. **Given** the need to debug the system, **When** the student uses ROS 2 tools, **Then** they can successfully identify and resolve communication issues.
3. **Given** the completed project, **When** the student demonstrates it, **Then** all communication patterns work as expected and the system meets requirements.

---

### Edge Cases

- What happens when nodes fail to start properly or encounter runtime errors?
- How does the system handle message type mismatches between publishers and subscribers?
- What if QoS settings are incompatible between publisher and subscriber?
- How does the student handle service timeouts or connection failures?
- What happens when multiple publishers send to the same topic or when multiple subscribers listen to one publisher?
- How does the student debug issues when nodes are distributed across different machines?
- What if resource constraints prevent proper node operation?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Book MUST provide detailed content for Section 1 (Understanding ROS 2 Nodes) with 1,300-1,700 words total
- **FR-002**: Book MUST provide detailed content for Section 2 (Topics - Publish/Subscribe Pattern) with 3,100-3,900 words total
- **FR-003**: Book MUST provide detailed content for Section 3 (Services - Request/Response Pattern) with 3,100-3,900 words total
- **FR-004**: Book MUST provide detailed content for Section 4 (Advanced Concepts) with 1,600-2,200 words total
- **FR-005**: Book MUST provide detailed content for Section 5 (Debugging and Tools) with 1,100-1,500 words total
- **FR-006**: Book MUST provide detailed content for Section 6 (Hands-On Project) with 1,500-2,000 words total
- **FR-007**: Book MUST provide detailed content for Section 7 (Exercises and Challenges) for 6 different exercises
- **FR-008**: Book MUST provide detailed content for Section 8 (Summary and Next Steps)
- **FR-009**: Book MUST include at least 18 complete code examples as specified in the requirements
- **FR-010**: Book MUST include 8-10 diagrams including architecture, flowcharts, and sequence diagrams
- **FR-011**: Book MUST specify estimated reading time of 2-3 hours and hands-on practice time of 4-6 hours
- **FR-012**: Book MUST explain all ROS 2 jargon when first introduced to students
- **FR-013**: Book MUST include at least 5-7 "💡 Pro Tip" callouts throughout the content
- **FR-014**: Book MUST include at least 5-7 "⚠️ Common Pitfall" warnings throughout the content
- **FR-015**: Book MUST include at least 5-7 "✅ Checkpoint" sections after major concepts
- **FR-016**: Book MUST use clear, simple language appropriate for students with basic Python knowledge
- **FR-017**: Book MUST provide analogies to explain complex ROS 2 concepts
- **FR-018**: Book MUST include practical examples for each ROS 2 CLI tool covered
- **FR-019**: Book MUST provide solutions or guidance for all exercises presented
- **FR-020**: Book MUST verify all code examples work in ROS 2 Iron environment

### Chapter-Specific Requirements

- **FR-021**: Section 1.1 (What is a Node?) MUST explain definition, purpose, node lifecycle, and when to create new nodes vs reusing existing ones (300-400 words)
- **FR-022**: Section 1.2 (Node Anatomy) MUST cover initialization, configuration, parameters, and include Code Example 1: Minimal node (400-500 words)
- **FR-023**: Section 1.3 (Creating Your First Node) MUST provide step-by-step tutorial with Code Example 2: Complete node with logging, running/testing instructions, common errors and solutions (600-800 words)
- **FR-024**: Section 2.1 (Introduction to Topics) MUST explain what topics are, publish-subscribe pattern, and use cases (300-400 words)
- **FR-025**: Section 2.2 (Message Types) MUST cover standard ROS 2 message types, string/int/float/custom messages, how to find and use them, and include Code Example 3: Exploring message types (400-500 words)
- **FR-026**: Section 2.3 (Creating a Publisher) MUST cover publisher node anatomy, include Code Example 4: String publisher, explain publishing rate/timing, QoS basics, and testing with ros2 topic echo (700-900 words)
- **FR-027**: Section 2.4 (Creating a Subscriber) MUST cover subscriber node anatomy, callback functions explanation, include Code Example 5: String subscriber, and testing publisher-subscriber communication (700-900 words)
- **FR-028**: Section 2.5 (Practical Example - Temperature Monitor) MUST provide real-world scenario with Code Example 6: Temperature publisher, Code Example 7: Temperature subscriber with alerts, running multiple nodes, and visualization with rqt_graph (1000-1200 words)
- **FR-029**: Section 3.1 (Understanding Services) MUST explain what services are, request-response pattern, and when to use services vs topics (300-400 words)
- **FR-030**: Section 3.2 (Service Types) MUST cover standard service types, service definition files (.srv), request/response structure, and include Code Example 8: Examining service types (400-500 words)
- **FR-031**: Section 3.3 (Creating a Service Server) MUST cover service server anatomy, include Code Example 9: AddTwoInts service server, explain handling requests/responses, and error handling (800-1000 words)
- **FR-032**: Section 3.4 (Creating a Service Client) MUST cover service client anatomy, include Code Example 10: AddTwoInts service client, explain synchronous vs asynchronous calls, and timeout handling (700-900 words)
- **FR-033**: Section 3.5 (Practical Example - Robot Controller) MUST provide real-world scenario with Code Example 11: Motion service server, Code Example 12: Motion service client, and integration testing (1000-1200 words)
- **FR-034**: Section 4.1 (Quality of Service) MUST explain QoS profiles, reliability/durability/history, include Code Example 13: Publisher with custom QoS, and explain when to use different settings (600-800 words)
- **FR-035**: Section 4.2 (Node Composition) MUST explain component nodes, benefits, and include Code Example 14: Composable node (500-700 words)
- **FR-036**: Section 4.3 (Lifecycle Nodes) MUST explain managed nodes, state transitions, and use cases (500-700 words)
- **FR-037**: Section 5.1 (ROS 2 CLI Tools) MUST cover ros2 node list/info, ros2 topic list/echo/pub, ros2 service list/call with practical examples (600-800 words)
- **FR-038**: Section 5.2 (Visualization Tools) MUST cover rqt_graph, rqt_console, rqt_plot for visualization (500-700 words)
- **FR-039**: Section 6 (Hands-On Project) MUST include Multi-Node Robot Telemetry System requirements, system architecture diagram, Code Examples 15-18, integration guide, and extension challenges (1500-2000 words)
- **FR-040**: Section 7 (Exercises and Challenges) MUST include 6 exercises at different difficulty levels: Basic Node Creation, Publisher-Subscriber Chain, Service-Based Calculator, Multi-Node System, Custom Message Types, and Design Challenge
- **FR-041**: Section 8 (Summary and Next Steps) MUST include key takeaways, concept map/mind map, Chapter 3 preview, additional resources (10-15 links), and FAQ (8-10 questions)

### Key Entities

- **Node**: A process that performs computation in ROS 2, containing publishers, subscribers, services, and other ROS graph components
- **Topic**: A communication channel for streaming data between nodes using publish/subscribe pattern
- **Publisher**: A node component that sends messages to a topic
- **Subscriber**: A node component that receives messages from a topic
- **Service**: A communication pattern for request-response interactions between nodes
- **Service Server**: A node component that receives requests and sends responses
- **Service Client**: A node component that sends requests and receives responses
- **Message Type**: Defined data structures used for communication between nodes (e.g., std_msgs/String, sensor_msgs/LaserScan)
- **Service Type**: Defined request/response structure for service communications (e.g., example_interfaces/AddTwoInts)
- **Quality of Service (QoS)**: Configuration settings that define communication behavior (reliability, durability, history, etc.)
- **Node Composition**: Running multiple node components within a single process
- **Lifecycle Node**: A managed node with explicit state transitions (unconfigured, inactive, active, finalized)
- **Code Example**: A complete, runnable code snippet demonstrating a specific concept
- **Exercise**: A practical task for students to apply concepts learned
- **Diagram**: A visual representation supporting the chapter's content

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 90% of students successfully complete the hands-on project by building a working multi-node robot telemetry system
- **SC-002**: Students complete the chapter content within 2-3 hours of reading time and 4-6 hours of hands-on practice as specified
- **SC-003**: 95% of students can independently create a publisher-subscriber pair that successfully communicate
- **SC-004**: 90% of students can create a service server and client that properly handle requests and responses
- **SC-005**: Students can achieve 80% accuracy on the exercises provided in Section 7
- **SC-006**: Students can use ROS 2 CLI tools (ros2 node, ros2 topic, ros2 service, rqt_graph) effectively for system inspection
- **SC-007**: Students demonstrate understanding of when to use topics vs services with 85% accuracy in scenario-based questions
- **SC-008**: 85% of students can explain Quality of Service concepts and select appropriate settings for common use cases
- **SC-009**: Students can implement QoS settings appropriately with 80% success rate in practical exercises
- **SC-010**: Students can debug basic ROS 2 communication issues with 80% success rate using provided tools