---
id: 12
title: Create Chapter 3 Spec
stage: spec
date: 2025-12-09
surface: agent
model: gpt-4o
feature: 003-chapter3-ros2-services-actions
branch: 003-chapter3-ros2-services-actions
user: ayana
command: /sp.specify
labels: [spec, ros2, chapter3, services, actions, education]
links:
  spec: specs/003-chapter3-ros2-services-actions/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/003-chapter3-ros2-services-actions/spec.md
 - specs/003-chapter3-ros2-services-actions/checklists/requirements.md
tests:
 - none
---

## Prompt

/sp.specify Based on the constitution for Chapter 3 "ROS 2 Services and Actions", create detailed specifications: 1. Section Breakdown: - Introduction (300 words) - Section 1: Understanding ROS 2 Services (800 words) - Section 2: Creating Service Servers and Clients (1000 words) - Section 3: Understanding ROS 2 Actions (800 words) - Section 4: Creating Action Servers and Clients (1000 words) - Section 5: Practical Comparison and Use Cases (600 words) - Summary and Next Steps (200 words) 2. For Each Section Specify: - Learning objectives - Key concepts and terminology - Code examples needed (with descriptions) - Diagrams/illustrations needed - Common pitfalls to address - Best practices to highlight 3. Code Examples Required: - Simple service server example (Python) - Simple service client example (Python) - Custom service interface definition - Action server example for humanoid robot arm control - Action client with feedback handling - Comparison example: Topic vs Service vs Action 4. Hands-on Exercises: - Exercise 1: Create a robot status service - Exercise 2: Build a humanoid gesture action server - Exercise 3: Integrate services into existing nodes 5. Technical Requirements: - ROS 2 Humble or later - Python 3.8+ - Required packages and dependencies - Hardware/simulation requirements 6. Visual Assets Needed: - Service communication diagram - Action state machine diagram - Comparison flowchart - Code execution timeline Format as a comprehensive specification document.

## Response snapshot

Created comprehensive specification for Chapter 3 "ROS 2 Services and Actions" with 5 user stories, 21 functional requirements, and 10 success criteria. The spec details all sections, code examples, exercises, and technical requirements.

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