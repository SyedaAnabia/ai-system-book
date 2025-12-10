---
id: 7
title: Create Chapter 2 Spec
stage: spec
date: 2025-12-09
surface: agent
model: gpt-4o
feature: 002-chapter2-nodes-topics-services
branch: 002-chapter2-nodes-topics-services
user: ayana
command: /sp.specify
labels: [spec, ros2, chapter2, education]
links:
  spec: specs/002-chapter2-nodes-topics-services/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-chapter2-nodes-topics-services/spec.md
 - specs/002-chapter2-nodes-topics-services/checklists/requirements.md
tests:
 - none
---

## Prompt

/sp.specify Based on the constitution, create detailed specifications for Chapter 2: "Nodes, Topics, and Services" CHAPTER STRUCTURE: Section 1: UNDERSTANDING ROS 2 NODES - Subsection 1.1: What is a Node? (300-400 words) * Definition and purpose * Node lifecycle * When to create new nodes vs reusing existing ones - Subsection 1.2: Node Anatomy (400-500 words) * Node initialization * Node configuration * Node parameters * Code Example 1: Minimal node (10-15 lines) - Subsection 1.3: Creating Your First Node (600-800 words) * Step-by-step tutorial * Code Example 2: Complete node with logging (30-40 lines) * Running and testing the node * Common errors and solutions Section 2: TOPICS - PUBLISH/SUBSCRIBE PATTERN - Subsection 2.1: Introduction to Topics (300-400 words) * What are topics? * Publish-Subscribe pattern explained * Use cases and scenarios - Subsection 2.2: Message Types (400-500 words) * Standard ROS 2 message types * String, Int, Float, Custom messages * How to find and use message types * Code Example 3: Exploring message types (CLI commands) - Subsection 2.3: Creating a Publisher (700-900 words) * Publisher node anatomy * Code Example 4: String publisher (40-50 lines) * Publishing rate and timing * Quality of Service (QoS) basics * Testing with ros2 topic echo - Subsection 2.4: Creating a Subscriber (700-900 words) * Subscriber node anatomy * Callback functions explained * Code Example 5: String subscriber (40-50 lines) * Testing publisher-subscriber communication - Subsection 2.5: Practical Example - Temperature Monitor (1000-1200 words) * Real-world scenario * Code Example 6: Temperature publisher (60-80 lines) * Code Example 7: Temperature subscriber with alerts (60-80 lines) * Running multiple nodes * Visualization with rqt_graph Section 3: SERVICES - REQUEST/RESPONSE PATTERN - Subsection 3.1: Understanding Services (300-400 words) * What are services? * Request-Response pattern * Topics vs Services - when to use what? - Subsection 3.2: Service Types (400-500 words) * Standard service types * Service definition files (.srv) * Request and Response structure * Code Example 8: Examining service types - Subsection 3.3: Creating a Service Server (800-1000 words) * Service server anatomy * Code Example 9: AddTwoInts service server (50-70 lines) * Handling requests * Sending responses * Error handling - Subsection 3.4: Creating a Service Client (700-900 words) * Service client anatomy * Code Example 10: AddTwoInts service client (50-70 lines) * Synchronous vs Asynchronous calls * Timeout handling - Subsection 3.5: Practical Example - Robot Controller (1000-1200 words) * Real-world scenario * Code Example 11: Motion service server (80-100 lines) * Code Example 12: Motion service client (60-80 lines) * Integration testing Section 4: ADVANCED CONCEPTS - Subsection 4.1: Quality of Service (QoS) (600-800 words) * QoS profiles explained * Reliability, Durability, History * Code Example 13: Publisher with custom QoS (30-40 lines) * When to use different QoS settings - Subsection 4.2: Node Composition (500-700 words) * Component nodes * Benefits of composition * Code Example 14: Composable node (40-50 lines) - Subsection 4.3: Lifecycle Nodes (500-700 words) * Managed nodes * State transitions * Use cases for lifecycle management Section 5: DEBUGGING AND TOOLS - Subsection 5.1: ROS 2 CLI Tools (600-800 words) * ros2 node list/info * ros2 topic list/echo/pub * ros2 service list/call * Practical examples for each - Subsection 5.2: Visualization Tools (500-700 words) * rqt_graph for system visualization * rqt_console for logging * rqt_plot for data visualization Section 6: HANDS-ON PROJECT - Project: Multi-Node Robot Telemetry System (1500-2000 words) * Requirements specification * System architecture diagram * Code Example 15: Sensor node (100-120 lines) * Code Example 16: Data processor node (100-120 lines) * Code Example 17: Control service (80-100 lines) * Code Example 18: Dashboard subscriber (80-100 lines) * Integration and testing guide * Extension challenges Section 7: EXERCISES AND CHALLENGES - Exercise 1: Basic Node Creation (Easy) - Exercise 2: Publisher-Subscriber Chain (Medium) - Exercise 3: Service-Based Calculator (Medium) - Exercise 4: Multi-Node Communication System (Hard) - Exercise 5: Custom Message Types (Hard) - Challenge Project: Design your own system (Advanced) Section 8: SUMMARY AND NEXT STEPS - Key Takeaways (5-7 bullet points) - Concept Map / Mind Map - Preview of Chapter 3 - Additional Resources (10-15 links) - FAQ (8-10 common questions) TECHNICAL SPECIFICATIONS: - Total word count: 10,000-12,000 words - Code examples: 18 complete examples - Diagrams needed: 8-10 (architecture, flowcharts, sequence diagrams) - Estimated reading time: 2-3 hours - Hands-on practice time: 4-6 hours STYLE REQUIREMENTS: - Use clear, simple language - Explain jargon when first introduced - Include "💡 Pro Tip" callouts (5-7 throughout) - Include "⚠️ Common Pitfall" warnings (5-7 throughout) - Use analogies to explain complex concepts - Add "✅ Checkpoint" sections after major concepts Format this as a detailed outline with word counts and specifications for each section.

## Response snapshot

Created a comprehensive specification for Chapter 2: "Nodes, Topics, and Services" with detailed user stories, functional requirements, and success criteria. The specification includes 5 user stories with acceptance scenarios and 41 functional requirements covering all sections of the chapter.

## Outcome

- ✅ Impact: Created complete chapter specification
- 🧪 Tests: none required
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: /sp.plan to create implementation plan
- 🧠 Reflection: Successfully detailed all chapter components with measurable outcomes

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): not applicable