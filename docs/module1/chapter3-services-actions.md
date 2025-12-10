# ROS 2 Services and Actions - Request-Response Communication

## Introduction (300 words)

Imagine a humanoid robot performing a complex task in an industrial setting. The robot's control system needs to make a critical decision: Is the environment safe for the next maneuver? Rather than continuously publishing sensor data (like topics would), the robot needs an immediate answer to this specific query. This is precisely where ROS 2 services come into play—they allow nodes to request specific information and receive immediate responses, enabling critical decision-making in robotics applications.

In this chapter, we delve deep into the synchronous communication patterns of ROS 2: services and actions. You'll learn how services facilitate request-response interactions and how actions manage long-running tasks with feedback. These communication patterns complement the publish-subscribe model you learned in Chapter 2, expanding your toolkit for building sophisticated robotic systems.

Understanding services and actions is fundamental to mastering ROS 2's communication architecture. Unlike topics, which provide asynchronous, one-way data flow, services enable synchronous, two-way communication where a client sends a request and awaits a specific response. Actions elevate this concept further by managing complex, long-duration tasks with ongoing feedback and the ability to cancel operations mid-execution—essential for humanoid robotics tasks like walking, manipulation, or navigation.

By the end of this chapter, you'll be able to implement service-based communication for on-demand queries and action-based systems for complex robot behaviors. You'll understand when to use topics, services, or actions, and how to create custom interfaces for your specific robotics applications. The chapter includes practical examples focused on humanoid robot applications, detailed code examples with clear explanations, and exercises that reinforce your learning. We'll explore real-world use cases that demonstrate how services and actions enable robots to interact intelligently with their environment and make timely decisions.

## Section 1: Understanding ROS 2 Services (800 words)

ROS 2 services implement a synchronous client-server communication pattern where one node (the service client) sends a request to another node (the service server) and waits for a response. This request-response paradigm is fundamentally different from the asynchronous publish-subscribe model you've already learned about. Services provide a direct method for nodes to request specific computation or information, making them ideal for operations that require immediate results or acknowledgments.

### Service Architecture: Client-Server Model

The service architecture in ROS 2 follows a traditional client-server model. A service server node offers a specific service by registering with the ROS graph under a unique service name. Multiple service client nodes can then connect to this service server to make requests. The key distinction is that the service client blocks execution until it receives a response from the server, creating a synchronous interaction.

The service interface is defined using `.srv` files, which specify two components: the request message format and the response message format. The request contains the parameters sent by the client to the server, while the response contains the computed result or status sent back from the server to the client. This structured format ensures that both clients and servers agree on the data format for communication.

### Service vs Topic Comparison

| Aspect | Topics | Services |
|--------|--------|----------|
| Communication Pattern | Publish-Subscribe | Request-Response |
| Synchronicity | Asynchronous | Synchronous |
| Data Direction | One-way (publisher → subscriber) | Two-way (client ↔ server) |
| Blocking | No - non-blocking | Yes - client waits for response |
| Use Case | Streaming data, sensor feeds | Configuration, queries, commands |
| Guaranteed Delivery | No | Yes |
| Performance | High-frequency, low-latency | Lower frequency, higher latency |

### When to Use Services

Services are ideal for scenarios demanding immediate responses:

- **Configuration queries**: Getting robot parameters, updating settings, or changing operational modes
- **State queries**: Requesting current robot status, battery level, or sensor calibration
- **Mode switching**: Changing robot operational states (idle, active, maintenance)
- **Short-duration tasks**: Calculations or operations that complete quickly without intermediate feedback

### Built-in ROS 2 Services Examples

ROS 2 provides several standard services through packages like `std_srvs` and `example_interfaces`. For instance, the `SetBool` service allows toggling a Boolean state, while `Trigger` enables activating specific actions with success/error responses. These serve as excellent templates for custom service design.

### Real-world Humanoid Robotics Examples

Consider these practical examples in humanoid robotics:

- **Get robot joint positions**: A client requests current joint angles for posture analysis
- **Set operating mode**: A client sends a request to switch the robot from "navigation" to "manipulation" mode
- **Emergency stop trigger**: A safety module sends an immediate request to halt all robot motion

Services ensure these critical operations receive immediate attention and confirmation, which is impossible with the fire-and-forget nature of topics.

⚠️ **Warning**: Misusing services for high-frequency data transmission can lead to performance bottlenecks since each request blocks the client until receiving a response. Use topics for continuous data streams.

💡 **Pro Tip**: Services are perfect for implementing RPC-style (Remote Procedure Call) interfaces where you need guaranteed delivery and immediate confirmation of operations.

## Section 2: Creating Service Servers and Clients (1000 words)

### Part A: Service Interfaces

Service interfaces in ROS 2 are defined by `.srv` files that specify both the request and response message structures. These files follow a specific format where the request fields are listed first (before the `---` separator), followed by the response fields. Standard service types are available in packages like `std_srvs` and `example_interfaces`, but custom services are essential for specialized robotics applications.

For example, let's create a custom service definition `CalculateTrajectory.srv` to handle trajectory calculation requests for humanoid robot arms:

```
# Request fields for calculating a trajectory
float64 start_position
float64 end_position
float64 duration
---
# Response fields containing the trajectory
bool success
string message
float64[] positions
float64[] velocities
```

This definition specifies that clients will send start/end positions and duration, and the server will respond with success status, a message, and arrays of positions and velocities for the trajectory.

Standard service types include:
- `SetBool`: Toggle a Boolean value
- `Trigger`: Perform an action with success/failure response
- `Empty`: A service with no request/response data
- `example_interfaces/srv/AddTwoInts`: Add two integers and return the result

Custom service definition syntax requires:
1. Request fields listed before the `---` separator
2. Response fields listed after the separator
3. Standard ROS message types (int32, float64, string, etc.)
4. Descriptive field names following ROS naming conventions

### Part B: Creating a Service Server

To create a service server, you inherit from the Node class and implement a service using the `create_service` method. Here's a detailed example of implementing the CalculateTrajectory service:

```python
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
from builtin_interfaces.msg import Duration as DurationMsg
import rclpy

# Assuming we have our custom message type
# This would be generated from our .srv file
from trajectory_msgs.srv import CalculateTrajectory


class TrajectoryCalculationServer(Node):
    def __init__(self):
        super().__init__('trajectory_calculation_server')
        
        # Create the service server
        self.service = self.create_service(
            CalculateTrajectory,
            'calculate_trajectory',
            self.calculate_trajectory_callback
        )
        
        self.get_logger().info('Trajectory calculation service is ready.')

    def calculate_trajectory_callback(self, request, response):
        """
        Callback function that processes the trajectory calculation request.
        
        Args:
            request: The incoming request with start_position, end_position, and duration
            response: The response object to be populated with results
            
        Returns:
            response: The filled response object with trajectory data
        """
        self.get_logger().info(
            f'Received trajectory request: start={request.start_position}, '
            f'end={request.end_position}, duration={request.duration}'
        )
        
        # Validate inputs
        if request.duration <= 0.0:
            response.success = False
            response.message = 'Duration must be positive'
            return response
        
        if abs(request.start_position - request.end_position) < 0.001:
            response.success = False
            response.message = 'Start and end positions are too close'
            return response

        # Calculate trajectory using a simple linear interpolation
        # with trapezoidal velocity profile (constant acceleration phase)
        num_points = 50  # Number of discrete trajectory points
        dt = request.duration / (num_points - 1)  # Time step
        
        # Generate time steps
        t_steps = np.linspace(0, request.duration, num_points)
        
        # Generate positions with linear interpolation
        positions = []
        velocities = []
        
        # Simple linear trajectory - in practice, this would use more complex motion planning
        for t in t_steps:
            # Linear interpolation between start and end positions
            ratio = t / request.duration
            pos = request.start_position + ratio * (request.end_position - request.start_position)
            
            # Constant velocity (in practice, would be a smooth profile)
            vel = (request.end_position - request.start_position) / request.duration
            
            positions.append(float(pos))
            velocities.append(float(vel))
        
        # Populate response
        response.success = True
        response.message = 'Trajectory calculated successfully'
        response.positions = positions
        response.velocities = velocities
        
        self.get_logger().info(
            f'Trajectory calculation completed: {len(positions)} points generated'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    trajectory_server = TrajectoryCalculationServer()
    
    try:
        rclpy.spin(trajectory_server)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

This server demonstrates key aspects of service implementation: input validation, processing logic, and response population. The service server registers the `calculate_trajectory` service name, which clients can use to make requests. The callback function processes each request and returns an appropriate response.

### Part C: Creating a Service Client

Service clients initiate requests to service servers and await responses. Here's an example client for the trajectory calculation service:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.srv import CalculateTrajectory


class TrajectoryClient(Node):
    def __init__(self):
        super().__init__('trajectory_client')
        
        # Create a client for the trajectory service
        self.cli = self.create_client(CalculateTrajectory, 'calculate_trajectory')
        
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for trajectory service...')
        
        self.req = CalculateTrajectory.Request()

    def send_request(self, start_pos, end_pos, duration):
        """
        Send a request to calculate a trajectory.
        
        Args:
            start_pos: Starting position for the trajectory
            end_pos: Ending position for the trajectory
            duration: Time duration for the trajectory execution
            
        Returns:
            Future object that will hold the response
        """
        self.req.start_position = start_pos
        self.req.end_position = end_pos
        self.req.duration = duration
        
        self.get_logger().info(
            f'Sending trajectory request: start={start_pos}, '
            f'end={end_pos}, duration={duration}'
        )
        
        # Asynchronously call the service
        future = self.cli.call_async(self.req)
        return future


def main(args=None):
    rclpy.init(args=args)
    
    trajectory_client = TrajectoryClient()
    
    # Send a request for trajectory calculation
    start_pos = 0.0
    end_pos = 1.5
    duration = 2.0
    
    future = trajectory_client.send_request(start_pos, end_pos, duration)
    
    try:
        # Wait for the response (blocking until response arrives)
        rclpy.spin_until_future_complete(trajectory_client, future)
        
        if future.result() is not None:
            response = future.result()
            
            if response.success:
                trajectory_client.get_logger().info(
                    f'Trajectory calculated successfully!'
                    f'Message: {response.message}'
                    f'Number of points: {len(response.positions)}'
                )
                
                # Print the first few trajectory points
                for i in range(min(5, len(response.positions))):
                    pos = response.positions[i]
                    vel = response.velocities[i]
                    trajectory_client.get_logger().info(
                        f'Point {i}: Position={pos:.3f}, Velocity={vel:.3f}'
                    )
                    
                if len(response.positions) > 5:
                    trajectory_client.get_logger().info('...')
            else:
                trajectory_client.get_logger().error(
                    f'Trajectory calculation failed: {response.message}'
                )
        else:
            trajectory_client.get_logger().error(
                f'Exception while calling service: {future.exception()}'
            )
    
    except KeyboardInterrupt:
        trajectory_client.get_logger().info('Interrupted during service call')
    finally:
        trajectory_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

This client demonstrates how to create a service client, wait for the service to be available, send a request, and handle the response. Notice the synchronous nature - the `spin_until_future_complete` call blocks until the response is received.

💡 **Pro Tip**: Use timeouts when calling services to avoid indefinite blocking if the service server becomes unavailable.

## Section 3: Understanding ROS 2 Actions (800 words)

While services perfectly handle simple request-response interactions, humanoid robotics and other complex robotic applications often require managing long-running tasks with intermediate feedback. This is where ROS 2 actions excel. Actions provide a communication pattern specifically designed for operations that take significant time to complete, allowing clients to monitor progress, receive continuous feedback, and even cancel tasks if needed.

### What Are Actions?

An action is a communication pattern that extends the service model to support long-running, goal-oriented tasks. Unlike services, which provide an immediate response to a request, actions enable nodes to:

- Send a goal to initiate a long-running task
- Receive continuous feedback during execution
- Get a final result when the task completes
- Cancel or preempt the task before completion
- Track the current state of the task

The action interface consists of three message types encapsulated in a single `.action` file:
- **Goal**: Defines the goal request sent to the action server
- **Feedback**: Defines the intermediate feedback messages during execution
- **Result**: Defines the final result returned upon completion

### Action State Machine

Each action follows a well-defined state machine that tracks the lifecycle of the goal:

1. **PENDING**: Goal accepted but not yet processed
2. **ACTIVE**: Goal is being processed
3. **PREEMPTED**: Goal was cancelled before completion
4. **SUCCEEDED**: Goal completed successfully
5. **ABORTED**: Goal failed to complete due to an error
6. **RECALLING**: Goal is being recalled
7. **REJECTED**: Goal was rejected and never processed
8. **LOST**: Connection to the action server was lost

The action client can monitor state transitions throughout the execution of the goal, enabling sophisticated interaction patterns.

### When to Use Actions

Actions are ideal for:
- **Navigation tasks**: Moving a robot to a specific location with path feedback
- **Manipulation tasks**: Controlling robot arms with joint position feedback
- **Calibration processes**: Long-running sensor calibration with progress updates
- **Data processing**: Computational tasks with progress feedback
- **Complex behaviors**: Multi-step robot behaviors requiring monitoring

### Action vs Service vs Topic Comparison

| Pattern | Communication | Latency | Feedback | Cancellation | Best For |
|---------|---------------|---------|----------|--------------|----------|
| Topics | Continuous stream | Low | No | No | Sensor data, status |
| Services | Request-Response | Medium | No | No | Configuration, queries |
| Actions | Goal-Feedback-Result | High | Yes | Yes | Long-running tasks, behaviors |

### Real-world Humanoid Robotics Examples

In humanoid robotics applications, actions are invaluable for complex tasks:

- **Walking locomotion**: A client sets a walking goal with target distance and speed, receiving continuous feedback about step count, balance status, and position
- **Arm manipulation**: A client sends a grasping goal for a specific object, getting feedback about the gripper position, force sensors, and grasp probability
- **Navigation**: A client requests a path to a destination, receiving continuous updates about the robot's location, path progress, and obstacle detection

### Action Architecture

The action architecture consists of:
- **Action Client**: Sends goals, receives feedback and results
- **Action Server**: Processes goals, sends feedback, returns results
- **Action Interface**: Defines goal, feedback, and result message types
- **Middleware**: Manages the communication between client and server

## Section 4: Creating Action Servers and Clients (1000 words)

### Part A: Action Interfaces

Action interfaces are defined using `.action` files that explicitly declare three message types: goal, result, and feedback. These files follow a strict format with three sections separated by `---` markers. The order is always goal (top), result (middle), and feedback (bottom).

Here's an example of a custom action definition `ExecuteHumanoidWalk.action` for controlling humanoid gait:

```
# Goal: Request to walk to a certain distance
float64 target_distance_meters
float64 walking_speed_mps
string gait_type  # 'bipedal', 'quadrupedal', etc.
---
# Result: Final outcome of the walking task
bool success
string message
float64 actual_distance_moved
int32 steps_taken
bool balance_maintained
---
# Feedback: Continuous updates during walking
float64 distance_traveled
float64 estimated_remaining_time
float64 balance_score
int32 steps_completed
string current_phase  # 'swing', 'stance', 'transition'
```

This action interface defines a walking task where a client sets a target distance and speed, receives continuous feedback about the robot's progress, and gets a final result with success status and statistics.

### Part B: Creating an Action Server

Creating an action server involves inheriting from Node and implementing the action server using the `rclpy.action` interfaces. Here's a detailed implementation of the humanoid walking action server:

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time
import threading
from math import sqrt

# Assuming our custom action type
from humanoid_locomotion_interfaces.action import ExecuteHumanoidWalk


class HumanoidWalkActionServer(Node):
    def __init__(self):
        super().__init__('humanoid_walk_action_server')

        # Create action server with callback group to handle multiple goals
        self._action_server = ActionServer(
            self,
            ExecuteHumanoidWalk,
            'execute_humanoid_walk',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.get_logger().info('Humanoid walk action server is ready.')

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info(
            f'Received walk goal: distance={goal_request.target_distance_meters}, '
            f'speed={goal_request.walking_speed_mps}, gait={goal_request.gait_type}'
        )

        # Validate the goal parameters
        if goal_request.target_distance_meters <= 0:
            self.get_logger().warn('Invalid target distance - rejecting goal')
            return GoalResponse.REJECT

        if goal_request.walking_speed_mps <= 0:
            self.get_logger().warn('Invalid walking speed - rejecting goal')
            return GoalResponse.REJECT

        # Accept the goal
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancellation request."""
        self.get_logger().info('Received request to cancel walk goal')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal and return the result."""
        self.get_logger().info('Executing walk goal...')

        # Get the goal request
        goal = goal_handle.request
        target_distance = goal.target_distance_meters
        walking_speed = goal.walking_speed_mps

        # Initialize feedback and result
        feedback_msg = ExecuteHumanoidWalk.Feedback()
        result_msg = ExecuteHumanoidWalk.Result()

        # Calculate estimated time
        estimated_time = target_distance / walking_speed
        estimated_remaining_time = estimated_time
        start_time = time.time()
        distance_traveled = 0.0
        steps_taken = 0
        steps_completed = 0

        # Main walking execution loop
        while distance_traveled < target_distance:
            # Check if the goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.success = False
                result_msg.message = 'Goal was canceled'
                result_msg.actual_distance_moved = distance_traveled
                result_msg.steps_taken = steps_taken
                result_msg.balance_maintained = False
                self.get_logger().info('Walk goal canceled')
                return result_msg

            # Simulate walking progress
            # In a real implementation, this would involve actual robot control
            elapsed = time.time() - start_time
            distance_traveled = min(target_distance, walking_speed * elapsed)

            # Calculate remaining time based on current progress
            estimated_remaining_time = max(0.0, estimated_time - elapsed)

            # Update steps taken (simulated - in reality this would come from sensors)
            steps_taken = int(distance_traveled / 0.5)  # Assume 0.5m per step
            steps_completed = steps_taken

            # Simulate balance score (0.0 to 1.0, with 1.0 being perfect balance)
            balance_score = 0.8 + 0.2 * (distance_traveled / target_distance)

            # Update feedback message
            feedback_msg.distance_traveled = distance_traveled
            feedback_msg.estimated_remaining_time = estimated_remaining_time
            feedback_msg.balance_score = balance_score
            feedback_msg.steps_completed = steps_completed

            # Determine current phase (this is simplified, real implementation would be more complex)
            if steps_completed % 3 == 0:
                feedback_msg.current_phase = 'stance'
            elif steps_completed % 3 == 1:
                feedback_msg.current_phase = 'swing'
            else:
                feedback_msg.current_phase = 'transition'

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                f'Walking progress: {distance_traveled:.2f}m / {target_distance:.2f}m '
                f'(phase: {feedback_msg.current_phase}, balance: {balance_score:.2f})'
            )

            # Sleep to simulate the time it takes for the robot to take steps
            time.sleep(0.5)

            # Small chance of balance loss (simulated)
            if balance_score < 0.3:
                goal_handle.abort()
                result_msg.success = False
                result_msg.message = 'Balance lost during walking'
                result_msg.actual_distance_moved = distance_traveled
                result_msg.steps_taken = steps_taken
                result_msg.balance_maintained = False
                self.get_logger().error('Aborting walk due to balance loss')
                return result_msg

        # Goal completed successfully
        if not goal_handle.is_cancel_requested:
            goal_handle.succeed()
            result_msg.success = True
            result_msg.message = f'Walk completed successfully. Traveled {distance_traveled:.2f} meters.'
            result_msg.actual_distance_moved = distance_traveled
            result_msg.steps_taken = steps_taken
            result_msg.balance_maintained = True
            self.get_logger().info(f'Walk goal succeeded: traveled {distance_traveled:.2f}m')

        return result_msg


def main(args=None):
    rclpy.init(args=args)

    walk_server = HumanoidWalkActionServer()

    try:
        # Use a multi-threaded executor to handle multiple goals if needed
        executor = MultiThreadedExecutor(num_threads=4)
        rclpy.spin(walk_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        walk_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

This action server demonstrates:
- Goal validation and acceptance/rejection
- Proper state management and cancellation handling
- Continuous feedback publishing during execution
- Final result generation when the task completes
- Thread-safe implementation for handling multiple goals

### Part C: Creating an Action Client

Action clients send goals to action servers and monitor progress. Here's an implementation of an action client for the walking action:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

# Assuming our custom action type
from humanoid_locomotion_interfaces.action import ExecuteHumanoidWalk


class HumanoidWalkActionClient(Node):
    def __init__(self):
        super().__init__('humanoid_walk_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            ExecuteHumanoidWalk,
            'execute_humanoid_walk'
        )

    def send_goal(self, target_distance, walking_speed, gait_type='bipedal'):
        """Send a goal to the action server."""
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create the goal message
        goal_msg = ExecuteHumanoidWalk.Goal()
        goal_msg.target_distance_meters = target_distance
        goal_msg.walking_speed_mps = walking_speed
        goal_msg.gait_type = gait_type

        self.get_logger().info(
            f'Sending walk goal: {target_distance}m at {walking_speed} m/s using {gait_type} gait'
        )

        # Send the goal and register callbacks for feedback and result
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Register callback for when the result is received
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

    def goal_response_callback(self, future):
        """Handle the result of sending the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server, waiting for result...')

        # Get the result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        self.get_logger().info(
            f'Received feedback: Traveled {feedback_msg.distance_traveled:.2f}m, '
            f'Remaining: {feedback_msg.estimated_remaining_time:.2f}s, '
            f'Balance: {feedback_msg.balance_score:.2f}, '
            f'Phase: {feedback_msg.current_phase}'
        )

    def get_result_callback(self, future):
        """Handle the final result from the action server."""
        result = future.result().result
        self.get_logger().info(f'Result received: {result.message}')

        if result.success:
            self.get_logger().info(
                f'Successfully moved {result.actual_distance_moved:.2f} meters '
                f'in {result.steps_taken} steps with maintained balance: {result.balance_maintained}'
            )
        else:
            self.get_logger().info('Walk action did not complete successfully')


def main(args=None):
    rclpy.init(args=args)

    walk_client = HumanoidWalkActionClient()

    # Send a goal to walk 3 meters at 0.5 m/s using bipedal gait
    future = walk_client.send_goal(3.0, 0.5, 'bipedal')

    try:
        # Spin to process callbacks
        rclpy.spin(walk_client)
    except KeyboardInterrupt:
        walk_client.get_logger().info('Interrupted during action execution')
    finally:
        walk_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

This action client demonstrates:
- Asynchronous goal sending with proper callbacks
- Feedback handling during execution
- Result handling when the action completes
- Proper client lifecycle management

## Section 5: Practical Comparison and Use Cases (600 words)

Understanding when to use topics, services, or actions is crucial for effective ROS 2 design. Each communication pattern serves different purposes and has distinct characteristics that make it suitable for specific use cases.

### Decision Framework for Communication Patterns

Use this framework to select the appropriate communication pattern:

- **Topics**: For streaming data, sensor feeds, and status updates where immediate response isn't needed
- **Services**: For simple, synchronous request-response interactions that complete quickly
- **Actions**: For long-running tasks that require feedback, monitoring, and potential cancellation

### Comprehensive Comparison

| Aspect | Topics | Services | Actions |
|--------|--------|----------|---------|
| **Purpose** | Streaming data | Request-response | Long-running tasks with feedback |
| **Synchronicity** | Asynchronous | Synchronous | Semi-synchronous |
| **Response Guarantee** | No | Yes | Yes (final result) |
| **Feedback** | No | No | Yes (continuous) |
| **Cancellation** | No | No | Yes |
| **Use Duration** | Continuous | Short-term | Long-term |
| **Communication Type** | One-way | Two-way | Goal-oriented |
| **State Tracking** | No | No | Yes (full state machine) |

### Real-World Robotics Use Cases

In humanoid robotics applications, selecting the right communication pattern is essential for effective robot control:

**Topics Use Cases:**
- Publishing sensor data (IMU readings, camera feeds, joint positions)
- Broadcasting robot state (current mode, battery level, active errors)
- Publishing control commands for joint controllers

**Services Use Cases:**
- Querying robot parameters (current joint limits, operational modes)
- Triggering immediate actions (emergency stop, zeroing joint encoders)
- Configuration changes (updating robot mode, setting calibration)

**Actions Use Cases:**
- Navigation to waypoints (with continuous pose feedback)
- Manipulation tasks (with continuous gripper state feedback)
- Walking locomotion (with continuous balance and step feedback)
- Complex multi-step behaviors (with continuous progress feedback)

### Performance Considerations

- **Topics**: Highest frequency/lowest latency, best for continuous data streams
- **Services**: Moderate frequency/latency, good for configuration and queries
- **Actions**: Lower frequency/higher latency, necessary for long-running tasks

### Integration Patterns

Often, robot systems combine multiple communication patterns. For example, a navigation system might use:
- **Topics** for continuous odometry and sensor data
- **Services** for setting navigation goals or querying robot status
- **Actions** for executing complex navigation tasks with progress feedback

This multi-pattern approach allows for robust, responsive robot systems that can handle various types of interactions effectively.

## Summary and Next Steps (200 words)

In this chapter, we've explored the synchronous communication patterns of ROS 2: services and actions. You've learned how services facilitate request-response interactions for immediate queries and configurations, and how actions manage long-running tasks with continuous feedback and cancellation capabilities.

We covered service interfaces defined in `.srv` files and implemented both service servers and clients with proper error handling. We examined action interfaces with their goal-feedback-result structure and built comprehensive action servers and clients with state management and feedback handling.

The comparison of topics vs services vs actions provides a clear decision framework for selecting the appropriate communication pattern based on your specific robotics application needs. Whether you're requesting immediate robot status (service) or commanding a long navigation task with progress feedback (action), you now have the knowledge to implement the correct pattern.

In the next chapter, we'll explore advanced ROS 2 concepts including Quality of Service settings, composition for efficient process management, and lifecycle nodes for complex system management. These advanced topics will further enhance your ability to design robust, scalable robotic systems.