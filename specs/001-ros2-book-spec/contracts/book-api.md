# Book API Contracts

This document outlines the API contracts for any platform functionality needed to support the AI Systems in the Physical World book.

## Book Content API

### Get Module Information
- **Endpoint**: `GET /api/modules/{module_id}`
- **Description**: Retrieve information about a specific module
- **Path Parameters**:
  - module_id: The unique identifier for the module (e.g., "module1-ros2")
- **Response**:
  - 200: Module details including title, description, and list of chapters
  - 404: Module not found

### Get Chapter Content
- **Endpoint**: `GET /api/modules/{module_id}/chapters/{chapter_id}`
- **Description**: Retrieve the content of a specific chapter
- **Path Parameters**:
  - module_id: The unique identifier for the module
  - chapter_id: The unique identifier for the chapter
- **Response**:
  - 200: Chapter content with learning objectives, text, code examples, and exercises
  - 404: Chapter not found

### Get Code Example
- **Endpoint**: `GET /api/code-examples/{example_id}`
- **Description**: Retrieve a specific code example with source code and metadata
- **Path Parameters**:
  - example_id: The unique identifier for the code example
- **Response**:
  - 200: Code example details including source code, language, dependencies, and expected output
  - 404: Code example not found

### Submit Exercise Solution
- **Endpoint**: `POST /api/exercises/{exercise_id}/solutions`
- **Description**: Submit a solution for a specific exercise for validation
- **Path Parameters**:
  - exercise_id: The unique identifier for the exercise
- **Request Body**: Solution code or description
- **Response**:
  - 200: Solution accepted and validated
  - 400: Invalid solution format
  - 404: Exercise not found

## Progress Tracking API

### Save Student Progress
- **Endpoint**: `POST /api/students/{student_id}/progress`
- **Description**: Save the progress of a student through the book
- **Path Parameters**:
  - student_id: The unique identifier for the student
- **Request Body**: Progress data including completed modules, chapters, and exercises
- **Response**:
  - 200: Progress saved successfully
  - 400: Invalid progress data

### Get Student Progress
- **Endpoint**: `GET /api/students/{student_id}/progress`
- **Description**: Retrieve the progress of a student through the book
- **Path Parameters**:
  - student_id: The unique identifier for the student
- **Response**:
  - 200: Progress data including completed modules, chapters, and exercises
  - 404: Student not found