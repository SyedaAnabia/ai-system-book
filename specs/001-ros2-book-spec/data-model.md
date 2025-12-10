# Data Model: AI Systems in the Physical World - Embodied Intelligence

**Feature**: 001-ros2-book-spec
**Date**: 2025-12-09

This document defines the key entities and their relationships for the educational content management of the book.

## Key Entities

### Chapter
- **Description**: A unit of educational content within a module
- **Fields**:
  - id: unique identifier for the chapter
  - title: descriptive title of the chapter
  - module: reference to the parent module
  - learning_objectives: array of learning objectives (3-5 points)
  - prerequisites: array of required knowledge/skills
  - estimated_reading_time: in minutes
  - content: the main content in markdown format
  - code_examples: array of code example references
  - exercises: array of practical exercise references
  - diagrams: array of visual aid references
  - summary: key takeaways from the chapter

### Module
- **Description**: A major section of the book containing related chapters
- **Fields**:
  - id: unique identifier for the module
  - title: descriptive title of the module
  - chapters: array of chapter references (3-4 chapters)
  - description: overview of the module's content
  - learning_outcomes: array of module-specific competencies
  - prerequisites: array of required knowledge/skills

### CodeExample
- **Description**: A runnable code snippet demonstrating a concept
- **Fields**:
  - id: unique identifier for the example
  - title: descriptive title of the example
  - language: programming language (Python, C++, etc.)
  - code: the actual code content
  - description: explanation of what the code demonstrates
  - chapter: reference to the chapter it belongs to
  - dependencies: list of required packages/libraries
  - expected_output: description of what the code should produce
  - testing_environment: requirements for running the code

### Exercise
- **Description**: A practical task for students to apply concepts learned
- **Fields**:
  - id: unique identifier for the exercise
  - title: descriptive title of the exercise
  - description: detailed instructions for the task
  - difficulty: level (beginner, intermediate, advanced)
  - chapter: reference to the chapter it belongs to
  - required_skills: list of skills needed to complete the exercise
  - solution: reference to the solution (for instructors)
  - estimated_completion_time: in minutes

### Diagram
- **Description**: A visual aid supporting the book's content
- **Fields**:
  - id: unique identifier for the diagram
  - title: descriptive title of the diagram
  - file_path: location of the visual file
  - alt_text: alternative text for accessibility
  - description: explanation of what the diagram illustrates
  - chapter: reference to the chapter it belongs to
  - type: category (flowchart, architecture, screenshot, etc.)

### LearningObjective
- **Description**: A specific, measurable outcome for student learning
- **Fields**:
  - id: unique identifier for the objective
  - statement: clear description of what students should understand
  - module: reference to the parent module
  - chapter: reference to the specific chapter (optional, if specific)
  - measurable: boolean indicating if the objective is testable

## Entity Relationships

```
Module (1) -- (3-4) Chapter (1) -- (n) CodeExample
Module (1) -- (3-4) Chapter (1) -- (n) Exercise
Module (1) -- (3-4) Chapter (1) -- (n) Diagram
Module (1) -- (n) LearningObjective
Chapter (1) -- (3-5) LearningObjective
```

### Validation Rules

Each entity has specific validation requirements based on the constitution:

1. **Chapter**:
   - Must have 3-5 learning objectives
   - Must specify estimated reading time
   - Must define prerequisites
   - Must include at least one code example
   - Must include at least one practical exercise

2. **CodeExample**:
   - Must be tested in the specified environment
   - Must include expected output or behavior
   - Must follow best practices for the target framework
   - Must be compatible with current stable versions of frameworks

3. **Diagram**:
   - Must have high-resolution output (at least 300 DPI)
   - Must be supported by appropriate alt text for accessibility
   - Must directly support concepts in the associated chapter

4. **Exercise**:
   - Must have a clear solution pathway
   - Must align with chapter learning objectives
   - Must be achievable with provided prerequisites

## State Transitions

### Chapter Lifecycle
- Draft → In Review → Approved → Published
- Each transition requires validation of completeness and accuracy

### Code Example Lifecycle
- Proposed → Developed → Tested → Documented → Approved
- Each transition includes verification in the target environment

### Diagram Lifecycle
- Concept → Design → Review → Approved → Published
- Each transition includes quality and accuracy checks