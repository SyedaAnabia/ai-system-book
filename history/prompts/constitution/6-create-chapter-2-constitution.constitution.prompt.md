---
id: 6
title: Create Chapter 2 Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: gpt-4o
feature: chapter2-constitution
branch: main
user: ayana
command: /sp.constitution
labels: [constitution, ros2, chapter, education]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/chapter2-constitution.md
tests:
 - none
---

## Prompt

/sp.constitution Create a chapter constitution for "Chapter 2: Nodes, Topics, and Services" in Module 1 (ROS 2 - The Robotic Nervous System).

CHAPTER CONTEXT:
- This is the second chapter in the ROS 2 module
- Readers have already completed Chapter 1 (Introduction to ROS 2)
- This chapter focuses on the core communication mechanisms in ROS 2
- Target audience: Students with basic Python knowledge and ROS 2 installed

DEFINE:

1. CHAPTER MISSION:
   - What is the primary goal of this chapter?
   - What should students be able to DO after completing it?

2. CORE PRINCIPLES:
   - What are the 3-4 fundamental concepts that MUST be covered?
   - What is the logical flow of concepts?

3. LEARNING OUTCOMES:
   - List 5-7 specific, measurable learning outcomes
   - Use action verbs: create, implement, explain, debug, analyze

4. CONTENT GUIDELINES:
   - Theory vs Practice ratio (e.g., 30% theory, 70% hands-on)
   - Number of code examples needed (minimum)
   - Types of visualizations required (diagrams, flowcharts, etc.)
   - Practical exercises count

5. PREREQUISITES:
   - What MUST students know before starting?
   - Which Chapter 1 concepts are essential?

6. SUCCESS CRITERIA:
   - How will we know students understood the material?
   - What can they build by the end?

Format the response as a structured document with clear sections.

## Response snapshot

Created a comprehensive chapter constitution for "Chapter 2: Nodes, Topics, and Services" with detailed sections covering core principles, content guidelines, learning outcomes, and governance structure.

## Outcome

- ✅ Impact: Created complete chapter constitution
- 🧪 Tests: none required
- 📁 Files: Created .specify/memory/chapter2-constitution.md
- 🔁 Next prompts: Begin development of Chapter 2 content using this constitution
- 🧠 Reflection: Successfully structured chapter-specific governance document

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): not applicable