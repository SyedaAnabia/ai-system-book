# Data Model: Chapter 3 - ROS 2 Services and Actions

**Feature**: 003-chapter3-ros2-services-actions
**Date**: 2025-12-09

This document defines the key entities and their relationships for the educational content management of Chapter 3.

## Key Entities

### Section
- **Description**: A major division of the chapter content
- **Fields**:
  - id: unique identifier for the section
  - title: descriptive title of the section
  - chapter: reference to the parent chapter
  - word_count: expected word count
  - estimated_reading_time: in minutes
  - content_type: category (theory, practical, exercise, etc.)
  - code_examples: array of code example references
  - diagrams: array of visual aid references
  - learning_objectives: array of learning objectives for the section

### CodeExample
- **Description**: A runnable code snippet demonstrating a concept
- **Fields**:
  - id: unique identifier for the example
  - title: descriptive title of the example
  - language: programming language (Python, C++, etc.)
  - code: the actual code content
  - description: explanation of what the code demonstrates
  - section: reference to the section it belongs to
  - dependencies: list of required packages/libraries
  - expected_output: description of what the code should produce
  - testing_environment: requirements for running the code
  - difficulty: level (beginner, intermediate, advanced)

### Diagram
- **Description**: A visual aid supporting the chapter's content
- **Fields**:
  - id: unique identifier for the diagram
  - title: descriptive title of the diagram
  - file_path: location of the visual file
  - alt_text: alternative text for accessibility
  - description: explanation of what the diagram illustrates
  - section: reference to the section it belongs to
  - type: category (architecture, flowchart, state-machine, etc.)

### Exercise
- **Description**: A practical task for students to apply concepts learned
- **Fields**:
  - id: unique identifier for the exercise
  - title: descriptive title of the exercise
  - description: detailed instructions for the task
  - difficulty: level (easy, medium, hard)
  - section: reference to the section it belongs to
  - required_skills: list of skills needed to complete the exercise
  - solution: reference to the solution (for instructors)
  - estimated_completion_time: in minutes

### LearningObjective
- **Description**: A specific, measurable outcome for student learning
- **Fields**:
  - id: unique identifier for the objective
  - statement: clear description of what students should understand
  - chapter: reference to the parent chapter
  - section: reference to the specific section (optional, if specific)
  - measurable: boolean indicating if the objective is testable

### ServiceInterface
- **Description**: Definition of a ROS 2 service interface
- **Fields**:
  - id: unique identifier for the service interface
  - name: name of the service interface
  - request_type: the request message structure
  - response_type: the response message structure
  - description: explanation of what the service does
  - section: reference to the section where it's explained
  - file_path: location of the .srv file

### ActionInterface
- **Description**: Definition of a ROS 2 action interface
- **Fields**:
  - id: unique identifier for the action interface
  - name: name of the action interface
  - goal_type: the goal message structure
  - result_type: the result message structure
  - feedback_type: the feedback message structure
  - description: explanation of what the action does
  - section: reference to the section where it's explained
  - file_path: location of the .action file

## Entity Relationships

```
Chapter (1) -- (n) Section (1) -- (n) CodeExample
Chapter (1) -- (n) Section (1) -- (n) Diagram
Chapter (1) -- (n) Section (1) -- (n) Exercise
Chapter (1) -- (n) LearningObjective
Section (1) -- (n) LearningObjective
Section (1) -- (n) ServiceInterface (for Section 2)
Section (1) -- (n) ActionInterface (for Section 4)
ServiceInterface (1) -- (n) CodeExample
ActionInterface (1) -- (n) CodeExample
```

### Validation Rules

Each entity has specific validation requirements based on the constitution:

1. **Section**:
   - Must have a defined word count
   - Must specify estimated reading time
   - Must include at least one code example
   - Must include at least one learning objective

2. **CodeExample**:
   - Must be tested in the specified environment
   - Must include expected output or behavior
   - Must follow best practices for the target framework
   - Must be compatible with ROS 2 Iron/Humble distribution

3. **Diagram**:
   - Must have high-resolution output (at least 300 DPI)
   - Must be supported by appropriate alt text for accessibility
   - Must directly support concepts in the associated section

4. **Exercise**:
   - Must have a clear solution pathway
   - Must align with section learning objectives
   - Must be achievable with provided prerequisites

5. **LearningObjective**:
   - Must be specific and measurable
   - Must align with overall chapter goals
   - Must be achievable through the provided content

6. **ServiceInterface**:
   - Must have valid .srv file structure
   - Must include proper request/response fields
   - Must be tested in ROS 2 environment

7. **ActionInterface**:
   - Must have valid .action file structure
   - Must include proper goal/result/feedback fields
   - Must be tested in ROS 2 environment

## State Transitions

### Section Lifecycle
- Draft → In Review → Approved → Published
- Each transition requires validation of completeness and accuracy

### Code Example Lifecycle
- Proposed → Developed → Tested → Documented → Approved
- Each transition includes verification in the target environment

### Diagram Lifecycle
- Concept → Design → Review → Approved → Published
- Each transition includes quality and accuracy checks

### Exercise Lifecycle
- Draft → Reviewed → Tested → Approved → Published
- Each transition includes validation by domain experts

### ServiceInterface Lifecycle
- Design → Implementation → Testing → Documentation → Publication
- Each transition includes functional validation in ROS 2