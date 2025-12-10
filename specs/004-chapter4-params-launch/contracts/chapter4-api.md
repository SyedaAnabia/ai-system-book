# Chapter 4 API Contracts: Student Progress Tracking

## Chapter Information API

### Get Chapter Details
- **Endpoint**: `GET /api/chapters/004-params-launch/details`
- **Description**: Retrieve information about Chapter 4 including sections, learning objectives, and exercises
- **Response**:
  - 200: Chapter information including title, description, sections, learning objectives, and exercises
  - 404: Chapter not found

### Submit Section Completion
- **Endpoint**: `POST /api/chapters/004-params-launch/sections/{sectionId}/complete`
- **Description**: Record that a student has completed a specific section
- **Path Parameters**:
  - sectionId: The unique identifier for the section (e.g., "sec1", "sec2.1", etc.)
- **Request Body**: Student ID and completion timestamp
- **Response**:
  - 200: Completion recorded successfully
  - 400: Invalid request format
  - 404: Section not found

## Code Example Validation API

### Validate Parameter Code
- **Endpoint**: `POST /api/validation/parameter-code`
- **Description**: Validate a ROS 2 parameter implementation for correctness
- **Request Body**: Code content and environment specification
- **Response**:
  - 200: Validation successful with validation report
  - 400: Validation failed with specific error details

### Validate Launch File
- **Endpoint**: `POST /api/validation/launch-file`
- **Description**: Validate a ROS 2 launch file for syntax and correctness
- **Request Body**: Content of the launch file
- **Response**:
  - 200: Validation successful with validation report
  - 400: Validation failed with specific error details

### Submit Exercise Solution
- **Endpoint**: `POST /api/chapters/004-params-launch/exercises/{exerciseId}/submit`
- **Description**: Submit a solution for a specific chapter exercise for validation
- **Path Parameters**:
  - exerciseId: The unique identifier for the exercise
- **Request Body**: Solution code or description
- **Response**:
  - 200: Solution accepted and validated
  - 400: Invalid solution format
  - 404: Exercise not found

## Learning Progress API

### Get Chapter Progress
- **Endpoint**: `GET /api/students/{studentId}/chapters/004-params-launch/progress`
- **Description**: Retrieve the progress of a student through Chapter 4
- **Path Parameters**:
  - studentId: The unique identifier for the student
- **Response**:
  - 200: Progress data including completed sections, exercises, and overall completion percentage
  - 404: Student not found