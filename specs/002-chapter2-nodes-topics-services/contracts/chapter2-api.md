# Student Progress Tracking API Contract

## Chapter 2 Completion API

### Get Chapter Information
- **Endpoint**: `GET /api/chapters/002-chapter2-nodes-topics-services`
- **Description**: Retrieve information about Chapter 2 including sections, learning objectives, and exercises
- **Response**:
  - 200: Chapter information including title, description, sections, learning objectives, and exercises
  - 404: Chapter not found

### Submit Section Completion
- **Endpoint**: `POST /api/chapters/002-chapter2-nodes-topics-services/sections/{section_id}/completion`
- **Description**: Record that a student has completed a specific section
- **Path Parameters**:
  - section_id: The unique identifier for the section (e.g., "section1", "section2.1", etc.)
- **Request Body**: Student ID and completion timestamp
- **Response**:
  - 200: Completion recorded successfully
  - 400: Invalid request format
  - 404: Section not found

### Submit Exercise Solution
- **Endpoint**: `POST /api/chapters/002-chapter2-nodes-topics-services/exercises/{exercise_id}/solutions`
- **Description**: Submit a solution for a specific exercise for validation
- **Path Parameters**:
  - exercise_id: The unique identifier for the exercise
- **Request Body**: Solution code or description
- **Response**:
  - 200: Solution accepted and validated
  - 400: Invalid solution format
  - 404: Exercise not found

### Get Chapter Progress
- **Endpoint**: `GET /api/chapters/002-chapter2-nodes-topics-services/progress/{student_id}`
- **Description**: Retrieve the progress of a student through Chapter 2
- **Path Parameters**:
  - student_id: The unique identifier for the student
- **Response**:
  - 200: Progress data including completed sections, exercises, and overall completion percentage
  - 404: Student not found