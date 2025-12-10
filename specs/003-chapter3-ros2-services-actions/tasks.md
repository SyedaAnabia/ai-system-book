---

description: "Task list for implementing Chapter 3: ROS 2 Services and Actions"
---

# Tasks: Chapter 3 - ROS 2 Services and Actions

**Input**: Design documents from `/specs/[003-chapter3-ros2-services-actions]/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `docs/`, `static/`, `src/` at repository root
- **Code Examples**: `static/code-examples/` organized by module
- **Images**: `static/img/` organized by module

<!--
  ============================================================================
  IMPLEMENTATION TASKS FOR CHAPTER 3: ROS 2 SERVICES AND ACTIONS
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create docs/module1-ros2/chapter3-services-actions.md file structure per implementation plan
- [ ] T002 [P] Create static/code-examples/module1-ros2/chapter3-services-actions/ directory
- [ ] T003 [P] Create static/img/module1-ros2/ directory structure for diagrams
- [ ] T004 Update docusaurus.config.ts with chapter 3 navigation
- [ ] T005 Update sidebars.ts to include chapter 3 in navigation menu
- [ ] T006 [P] Set up code highlighting for Python and ROS 2 files in docusaurus

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T007 Create chapter 3 directory structure in static/code-examples/module1-ros2/
- [ ] T008 [P] Set up ROS 2 Iron/Humble development environment for testing
- [ ] T009 Configure syntax highlighting for Python, srv files, and action files in Docusaurus
- [ ] T010 [P] Create standardized section template following book constitution
- [ ] T011 Create checklist for chapter completion based on constitution requirements

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Student Understands ROS 2 Services (Priority: P1) 🎯 MVP

**Goal**: Students can explain the difference between services and topics, implement a simple service server and client, and know when to use services vs topics.

**Independent Test**: Students can explain the difference between services and topics, implement a simple service server and client, and know when to use services vs topics.

### Implementation for User Story 1

- [X] T012 [US1] Write Introduction section (300 words) in docs/module1-ros2/chapter3-services-actions.md
- [X] T013 [P] [US1] Write Section 1.1: Understanding ROS 2 Services (800 words) in docs/module1-ros2/chapter3-services-actions.md
- [X] T014 [P] [US1] Write Section 1.2: Service vs Topics comparison in docs/module1-ros2/chapter3-services-actions.md
- [X] T015 [US1] Add learning objectives to Section 1 in docs/module1-ros2/chapter3-services-actions.md
- [X] T016 [US1] Define key concepts and terminology for services in docs/module1-ros2/chapter3-services-actions.md
- [X] T017 [US1] Identify common pitfalls about services in docs/module1-ros2/chapter3-services-actions.md
- [X] T018 [US1] Highlight best practices for services in docs/module1-ros2/chapter3-services-actions.md
- [ ] T019 [P] [US1] Create service communication diagram in static/img/module1-ros2/service-communication-diagram.png
- [X] T020 [US1] Create simple service server example in static/code-examples/module1-ros2/chapter3-services-actions/simple_service_server.py
- [X] T021 [US1] Create simple service client example in static/code-examples/module1-ros2/chapter3-services-actions/simple_service_client.py
- [ ] T022 [US1] Test service examples in ROS 2 Iron/Humble environment
- [ ] T023 [US1] Document expected output for service examples
- [ ] T024 [US1] Verify service appears in `ros2 service list` command output
- [X] T025 [US1] Add summary section for Section 1 in docs/module1-ros2/chapter3-services-actions.md

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Student Creates Service Servers and Clients (Priority: P2)

**Goal**: Students can create both service servers and clients that communicate effectively with proper error handling and response management.

**Independent Test**: Students can create both service servers and clients that communicate effectively with proper error handling and response management.

### Implementation for User Story 2

- [X] T026 [US2] Write Section 2: Creating Service Servers and Clients (1000 words) in docs/module1-ros2/chapter3-services-actions.md
- [X] T027 [P] [US2] Write guide for creating service servers in docs/module1-ros2/chapter3-services-actions.md
- [X] T028 [P] [US2] Write guide for creating service clients in docs/module1-ros2/chapter3-services-actions.md
- [X] T029 [US2] Add learning objectives to Section 2 in docs/module1-ros2/chapter3-services-actions.md
- [X] T030 [US2] Document error handling in services in docs/module1-ros2/chapter3-services-actions.md
- [X] T031 [US2] Explain custom service interface definition in docs/module1-ros2/chapter3-services-actions.md
- [ ] T032 [P] [US2] Create custom service interface AddTwoInts.srv in static/code-examples/module1-ros2/chapter3-services-actions/custom_service_definition.srv
- [ ] T033 [US2] Implement detailed service server example in static/code-examples/module1-ros2/chapter3-services-actions/service_server_detailed.py
- [ ] T034 [US2] Implement detailed service client example in static/code-examples/module1-ros2/chapter3-services-actions/service_client_detailed.py
- [ ] T035 [US2] Test both detailed examples in ROS 2 environment
- [ ] T036 [US2] Verify service communication between detailed server and client
- [ ] T037 [US2] Document troubleshooting for service implementations
- [X] T038 [US2] Add summary section for Section 2 in docs/module1-ros2/chapter3-services-actions.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Student Understands ROS 2 Actions (Priority: P3)

**Goal**: Students can explain when to use actions vs services vs topics, and understand the goal-feedback-result communication pattern.

**Independent Test**: Students can explain when to use actions vs services vs topics, and understand the goal-feedback-result communication pattern.

### Implementation for User Story 3

- [X] T039 [US3] Write Section 3: Understanding ROS 2 Actions (800 words) in docs/module1-ros2/chapter3-services-actions.md
- [X] T040 [P] [US3] Explain goal-feedback-result pattern in docs/module1-ros2/chapter3-services-actions.md
- [X] T041 [P] [US3] Describe action state machine in docs/module1-ros2/chapter3-services-actions.md
- [X] T042 [US3] Add learning objectives to Section 3 in docs/module1-ros2/chapter3-services-actions.md
- [X] T043 [US3] Compare actions to services and topics in docs/module1-ros2/chapter3-services-actions.md
- [X] T044 [US3] Cover use cases for long-running tasks in docs/module1-ros2/chapter3-services-actions.md
- [ ] T045 [P] [US3] Create action state machine diagram in static/img/module1-ros2/action-state-machine-diagram.png
- [X] T046 [P] [US3] Create basic action server example in static/code-examples/module1-ros2/chapter3-services-actions/basic_action_server.py
- [X] T047 [P] [US3] Create basic action client example in static/code-examples/module1-ros2/chapter3-services-actions/basic_action_client.py
- [ ] T048 [US3] Test basic action communication
- [ ] T049 [US3] Document action states and transitions
- [X] T050 [US3] Add summary section for Section 3 in docs/module1-ros2/chapter3-services-actions.md

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Student Creates Action Servers and Clients (Priority: P4)

**Goal**: Students can create action servers for humanoid robot tasks and clients that properly monitor and control these tasks.

**Independent Test**: Students can create action servers for humanoid robot tasks and clients that properly monitor and control these tasks.

### Implementation for User Story 4

- [X] T051 [US4] Write Section 4: Creating Action Servers and Clients (1000 words) in docs/module1-ros2/chapter3-services-actions.md
- [X] T052 [P] [US4] Write guide for creating action servers in docs/module1-ros2/chapter3-services-actions.md
- [X] T053 [P] [US4] Write guide for creating action clients in docs/module1-ros2/chapter3-services-actions.md
- [X] T054 [US4] Add learning objectives to Section 4 in docs/module1-ros2/chapter3-services-actions.md
- [X] T055 [US4] Explain feedback handling in actions in docs/module1-ros2/chapter3-services-actions.md
- [X] T056 [US4] Describe cancellation and preemption in docs/module1-ros2/chapter3-services-actions.md
- [ ] T057 [P] [US4] Create humanoid gesture action server in static/code-examples/module1-ros2/chapter3-services-actions/humanoid_gesture_action_server.py
- [X] T058 [US4] Create action client with feedback handling in static/code-examples/module1-ros2/chapter3-services-actions/action_client_with_feedback.py
- [ ] T059 [US4] Test action server and client communication
- [ ] T060 [US4] Verify feedback messages during action execution
- [ ] T061 [US4] Test action cancellation functionality
- [X] T062 [US4] Add summary section for Section 4 in docs/module1-ros2/chapter3-services-actions.md

**Checkpoint**: At this point, User Stories 1-4 should all work independently

---

## Phase 7: User Story 5 - Student Compares Communication Patterns (Priority: P5)

**Goal**: Students can identify the appropriate communication pattern for different robotics scenarios and explain their reasoning.

**Independent Test**: Students can identify the appropriate communication pattern for different robotics scenarios and explain their reasoning.

### Implementation for User Story 5

- [X] T063 [US5] Write Section 5: Practical Comparison and Use Cases (600 words) in docs/module1-ros2/chapter3-services-actions.md
- [X] T064 [P] [US5] Create comprehensive comparison of topics vs services vs actions in docs/module1-ros2/chapter3-services-actions.md
- [X] T065 [US5] Develop decision framework for choosing communication patterns in docs/module1-ros2/chapter3-services-actions.md
- [X] T066 [US5] Document real-world robotics use cases in docs/module1-ros2/chapter3-services-actions.md
- [X] T067 [US5] Include performance considerations in docs/module1-ros2/chapter3-services-actions.md
- [ ] T068 [P] [US5] Create comparison flowchart diagram in static/img/module1-ros2/comparison-flowchart.png
- [X] T069 [P] [US5] Create comparison example: Topic vs Service vs Action in static/code-examples/module1-ros2/chapter3-services-actions/topic_service_action_comparison.py
- [ ] T070 [US5] Test all three communication patterns in comparison example
- [ ] T071 [US5] Document trade-offs between patterns
- [X] T072 [US5] Write Summary and Next Steps section (200 words) in docs/module1-ros2/chapter3-services-actions.md

**Checkpoint**: At this point, User Stories 1-5 should all work independently

---

## Phase 8: Exercises Development

**Goal**: Complete all hands-on exercises for practical application

### Implementation for Exercises

- [X] T073 Create Exercise 1: Robot status service in docs/module1-ros2/chapter3-services-actions.md
- [X] T074 [P] Create robot status service implementation in static/code-examples/module1-ros2/chapter3-services-actions/robot_status_service.py
- [X] T075 Create Exercise 2: Humanoid gesture action server in docs/module1-ros2/chapter3-services-actions.md
- [X] T076 [P] Create humanoid gesture action server implementation in static/code-examples/module1-ros2/chapter3-services-actions/humanoid_gesture_action_server.py
- [X] T077 Create Exercise 3: Service integration into existing nodes in docs/module1-ros2/chapter3-services-actions.md
- [ ] T078 [P] Create service integration example in static/code-examples/module1-ros2/chapter3-services-actions/service_integration_example.py
- [ ] T079 Test all exercise solutions in ROS 2 environment
- [ ] T080 Document solutions for all exercises

---

## Phase 9: Quality Assurance & Polishing

**Goal**: Ensure content quality and accuracy before publication

### Implementation for QA

- [X] T081 [P] Technical review of code examples accuracy
- [X] T082 [P] Grammar and clarity review of content
- [ ] T083 Test all code examples on fresh ROS 2 installation
- [ ] T084 Verify all diagrams render correctly in documentation
- [X] T085 Cross-check references to chapters 1 & 2
- [X] T086 [P] Update chapter to include forward references to chapter 4
- [ ] T087 Verify all links and cross-references work correctly
- [ ] T088 Create code execution timeline diagram in static/img/module1-ros2/code-execution-timeline.png
- [ ] T089 Add accessibility alt-text to all diagrams
- [ ] T090 Verify all code examples run without modification in target environment
- [ ] T091 Update README.md with instructions for chapter 3 examples
- [X] T092 Final proofreading of entire chapter content

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4 → P5)
- **Exercises (Phase 8)**: Depends on all user stories completion
- **QA (Phase 9)**: Depends on all content completion

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 concepts but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Builds on US1 concepts but should be independently testable
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Builds on US3 concepts but should be independently testable
- **User Story 5 (P5)**: Can start after Foundational (Phase 2) - Integrates concepts from all previous stories

### Within Each User Story

- Content before code examples
- Code examples before diagrams
- All content must be tested in appropriate environments
- Technical review before completion
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All diagrams for a user story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members after foundational phase

---

## Parallel Example: User Story 1

```bash
# Launch all writing tasks for User Story 1 together:
T013: Write Section 1.1: Understanding ROS 2 Services (800 words) in docs/module1-ros2/chapter3-services-actions.md
T014: Write Section 1.2: Service vs Topics comparison in docs/module1-ros2/chapter3-services-actions.md

# Launch all code examples for User Story 1 together:
T020: Create simple service server example in static/code-examples/module1-ros2/chapter3-services-actions/simple_service_server.py
T021: Create simple service client example in static/code-examples/module1-ros2/chapter3-services-actions/simple_service_client.py

# Launch all diagrams for User Story 1 together:
T019: Create service communication diagram in static/img/module1-ros2/service-communication-diagram.png
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 - Student Understands ROS 2 Services
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Add User Story 5 → Test independently → Deploy/Demo
7. Add Exercises → Test independently → Deploy/Demo
8. Add QA → Test independently → Deploy/Demo
9. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: Focus on User Stories 1-2 (Services)
   - Developer B: Focus on User Stories 3-4 (Actions)
   - Developer C: Focus on User Story 5 (Comparisons) + Exercises
   - Developer D: Focus on QA and Polishing
3. Stories complete and integrate independently
4. Final integration team handles cross-module consistency

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- All code examples must be tested in actual ROS 2 Iron/Humble environment
- Each section must follow the constitution requirements (learning objectives, exercises, etc.)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Verify all content aligns with the book's constitution principles (hands-on learning, progressive complexity)