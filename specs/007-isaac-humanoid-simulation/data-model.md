# Data Model: Isaac Humanoid Simulation and Navigation System

## Overview
This document defines the key data structures and entities for the Isaac Humanoid Simulation and Navigation System, covering simulation environments, humanoid robot models, sensors, navigation plans, and SLAM maps.

## Key Entities

### 1. VirtualEnvironment
Represents 3D simulation spaces with physics properties, lighting, and objects that affect robot behavior.

**Fields**:
- `id` (UUID): Unique identifier for the environment
- `name` (String): Human-readable name for the environment
- `description` (String): Detailed description of the environment
- `physics_properties` (PhysicsProperties): Physics parameters for the environment
- `lighting_config` (LightingConfig): Lighting setup for the environment
- `objects` (List<SimulatedObject>): List of objects in the environment
- `metadata` (Map<String, Any>): Additional metadata about the environment

**Validations**:
- `name` must be unique within the system
- `physics_properties` must include gravity, friction, and restitution coefficients
- `objects` list must contain at least one obstacle

**State Transitions**:
- `created` → `loading` → `ready` → `active` → `destroyed`

### 2. PhysicsProperties
Defines the physics parameters for simulation environments.

**Fields**:
- `gravity` (Vector3): Gravity vector (x, y, z) in m/s²
- `friction_coefficient` (Float): Static friction coefficient
- `restitution_coefficient` (Float): Bounce/restitution coefficient
- `linear_damping` (Float): Linear motion damping factor
- `angular_damping` (Float): Angular rotation damping factor

**Validations**:
- `gravity.z` should typically be negative (downward force)
- Coefficients must be between 0.0 and 1.0

### 3. LightingConfig
Configuration for the lighting in virtual environments.

**Fields**:
- `ambient_light` (ColorRGBA): Ambient light color and intensity
- `directional_lights` (List<DirectionalLight>): Directional lights in the scene
- `point_lights` (List<PointLight>): Point lights in the scene
- `environment_map` (String): Path to HDR environment map

### 4. SimulatedObject
Represents objects within the virtual environment.

**Fields**:
- `id` (UUID): Unique identifier for the object
- `name` (String): Name of the object
- `mesh_path` (String): Path to the object's 3D mesh
- `position` (Vector3): Position in world coordinates
- `rotation` (Quaternion): Orientation as quaternion
- `scale` (Vector3): Scale factors along x, y, z axes
- `mass` (Float): Mass in kilograms
- `collider_type` (ColliderType): Type of collision shape (box, sphere, capsule, mesh)
- `material_properties` (MaterialProperties): Properties affecting appearance and physics

### 5. HumanoidRobotModel
Represents the physical robot with joints, sensors, and kinematic properties for simulation.

**Fields**:
- `id` (UUID): Unique identifier for the robot model
- `name` (String): Name of the robot model
- `urdf_path` (String): Path to URDF file describing the robot
- `sdf_path` (String): Path to SDF file describing the robot (alternative format)
- `links` (List<Link>): Links that form the robot's body
- `joints` (List<Joint>): Joints connecting the robot's links
- `actuators` (List<Actuator>): Actuators controlling the joints
- `sensors` (List<SimulatedSensor>): Sensors attached to the robot
- `kinematic_chain` (KinematicChain): Structure of the robot's kinematic chain
- `balance_controller` (BalanceController): Controller for maintaining balance
- `locomotion_controller` (LocomotionController): Controller for walking/movement
- `collision_geometry` (List<CollisionGeometry>): Collision geometry definitions

**Validations**:
- Must have at least 2 legs defined in the kinematic chain for bipedal locomotion
- Joint limits must be within safe operational range
- The center of mass must remain within stability region during movements

**State Transitions**:
- `created` → `initialized` → `calibrated` → `active` → `paused` → `error`

### 6. Link
Individual rigid body part of the robot.

**Fields**:
- `id` (UUID): Unique identifier for the link
- `name` (String): Name of the link
- `inertial_properties` (InertialProperties): Mass, center of mass, and inertia matrix
- `visual_mesh` (MeshVisual): Visual representation
- `collision_geometry` (CollisionGeometry): Collision shape and properties
- `parent_joint` (String): Name of the parent joint connecting to this link

### 7. Joint
Connection between two links allowing relative motion.

**Fields**:
- `id` (UUID): Unique identifier for the joint
- `name` (String): Name of the joint
- `joint_type` (JointType): Type (revolute, prismatic, fixed, continuous, etc.)
- `parent_link` (String): Name of the parent link
- `child_link` (String): Name of the child link
- `origin_transform` (Transform): Transform from parent to child in zero position
- `axis` (Vector3): Joint axis of motion
- `limits` (JointLimits): Motion limits for the joint
- `dynamics` (JointDynamics): Friction and damping parameters

### 8. JointLimits
Defines the motion limits for a joint.

**Fields**:
- `lower` (Float): Lower limit in radians or meters
- `upper` (Float): Upper limit in radians or meters
- `effort` (Float): Maximum effort (torque or force)
- `velocity` (Float): Maximum velocity

### 9. SimulatedSensor
Digital representations of RGB cameras, depth sensors, and LiDAR with realistic noise and limitations.

**Fields**:
- `id` (UUID): Unique identifier for the sensor
- `name` (String): Name of the sensor
- `sensor_type` (SensorType): Type (RGB_CAMERA, DEPTH_CAMERA, LIDAR, IMU, etc.)
- `parent_link` (String): Link to which the sensor is attached
- `transform` (Transform): Transform from parent link to sensor frame
- `noise_model` (NoiseModel): Noise characteristics of the sensor
- `configuration` (SensorConfiguration): Specific configuration for sensor type
- `status` (SensorStatus): Current operational status

**Validations**:
- Transform must place sensor within or on the parent link
- Configuration must match the sensor type

### 10. SensorConfiguration
Configuration specific to different sensor types.

**For RGB Cameras**:
- `resolution` (Resolution): Width and height in pixels
- `fov_horizontal` (Float): Horizontal field of view in degrees
- `bit_depth` (Int): Bits per pixel
- `frame_rate` (Int): Frames per second
- `compression_format` (CompressionFormat): Image compression format

**For Depth Cameras**:
- `resolution` (Resolution): Width and height in pixels
- `fov_horizontal` (Float): Horizontal field of view in degrees
- `min_range` (Float): Minimum sensing distance in meters
- `max_range` (Float): Maximum sensing distance in meters
- `frame_rate` (Int): Frames per second
- `depth_precision` (Float): Depth precision in meters

**For LiDAR**:
- `range_min` (Float): Minimum range measurement
- `range_max` (Float): Maximum range measurement
- `angle_min` (Float): Minimum angle of scan
- `angle_max` (Float): Maximum angle of scan
- `angle_increment` (Float): Angular resolution of scan
- `time_increment` (Float): Time between measurements
- `scan_time` (Float): Time between scans
- `number_of_beams` (Int): Number of beams in the LiDAR
- `frame_rate` (Int): Scans per second

### 11. NoiseModel
Defines the noise characteristics for simulated sensors.

**Fields**:
- `noise_type` (NoiseType): Type of noise (gaussian, salt_and_pepper, etc.)
- `mean` (Float): Mean of the noise distribution
- `variance` (Float): Variance of the noise distribution
- `bias` (Float): Systematic bias in measurements
- `dropout_probability` (Float): Probability of complete measurement dropout

### 12. NavigationPlan
Path and trajectory data generated by Nav2 algorithms for robot movement.

**Fields**:
- `id` (UUID): Unique identifier for the navigation plan
- `robot_id` (UUID): ID of the robot this plan is for
- `goal_position` (Vector3): Target position in world coordinates
- `waypoints` (List<Vector3>): Sequence of waypoints forming the path
- `trajectories` (List<TrajectoryPoint>): Smooth trajectory points with timing
- `planned_at` (DateTime): Timestamp when the plan was created
- `valid_until` (DateTime): Timestamp when the plan expires
- `path_cost` (Float): Estimated cost of the path
- `constraints` (PathConstraints): Constraints applied during planning
- `status` (PlanStatus): Current status of the plan

**Validations**:
- Waypoints must be connected without collisions with known obstacles
- Goal position must be reachable given robot kinematics
- Validity period should be reasonable for dynamic environments

**State Transitions**:
- `created` → `validating` → `confirmed` → `executing` → `completed`/`cancelled`/`failed`

### 13. TrajectoryPoint
A point in the smooth trajectory with timing and motion information.

**Fields**:
- `position` (Vector3): Position in world coordinates
- `orientation` (Quaternion): Orientation at this point
- `linear_velocity` (Vector3): Linear velocity vector
- `angular_velocity` (Vector3): Angular velocity vector
- `timestamp` (Float): Time offset from plan start
- `foot_step_info` (FootStepInfo): Information about foot placement (for humanoid)

### 14. SLAMMap
Simultaneous localization and mapping data structures containing environmental information and robot pose estimates.

**Fields**:
- `id` (UUID): Unique identifier for the SLAM map
- `map_type` (MapType): Type (occupancy_grid, point_cloud, octree, etc.)
- `origin_transform` (Transform): Transform to world coordinate system
- `resolution` (Float): Resolution in meters per cell (for grid maps)
- `dimensions` (Vector3): Dimensions of the map in meters
- `data` (MapData): Actual map data depending on map type
- `poses_history` (List<PoseStamped>): Historical poses of the robot
- `landmarks` (List<Landmark>): Identified landmarks in the environment
- `last_updated` (DateTime): Timestamp of last update
- `confidence_map` (ConfidenceMap): Confidence values for each map element
- `loop_closure_data` (LoopClosureData): Information for loop closure

**State Transitions**:
- `initializing` → `mapping` → `optimized` → `updating` → `frozen`

### 15. PoseStamped
A pose with associated timestamp and frame information.

**Fields**:
- `pose` (Pose): Position (x, y, z) and orientation (quaternion)
- `timestamp` (DateTime): Time of the pose measurement
- `frame_id` (String): Coordinate frame in which the pose is expressed

### 16. Landmark
A distinctive feature in the environment identified during SLAM.

**Fields**:
- `id` (UUID): Unique identifier for the landmark
- `position` (Vector3): 3D position of the landmark
- `descriptor` (Descriptor): Feature descriptor for landmark identification
- `observation_count` (Int): Number of times this landmark was observed
- `first_observed` (DateTime): When the landmark was first seen
- `last_observed` (DateTime): When the landmark was last seen
- `covariance` (Matrix6x6): Uncertainty in the landmark position

### 17. SyntheticDataset
Container for synthetic data generated for AI model training.

**Fields**:
- `id` (UUID): Unique identifier for the dataset
- `name` (String): Name of the dataset
- `description` (String): Description of the dataset content
- `size` (UInt64): Size of the dataset in bytes
- `generated_at` (DateTime): When the dataset was generated
- `scenes_used` (List<String>): Scenes used for data generation
- `sensors_used` (List<SensorType>): Sensors that contributed to the data
- `annotation_types` (List<AnnotationType>): Types of annotations included
- `statistics` (DatasetStatistics): Statistical information about the content
- `files` (List<DatasetFile>): List of files in the dataset

### 18. DatasetFile
Reference to an individual file within a synthetic dataset.

**Fields**:
- `id` (UUID): Unique identifier for the file
- `filename` (String): Name of the file
- `filepath` (String): Path to the file
- `file_type` (FileType): Type of content (RGB_IMAGE, DEPTH_MAP, SEGMENTATION_MASK)
- `associated_annotations` (List<AnnotationFile>): Associated annotation files
- `scene_metadata` (SceneMetadata): Metadata about the scene when captured
- `sensor_metadata` (SensorMetadata): Metadata about the sensor when captured

### 19. SceneMetadata
Metadata about the scene when synthetic data was captured.

**Fields**:
- `environment_id` (String): ID of the virtual environment
- `object_positions` (List<ObjectPosition>): Positions of objects in the scene
- `lighting_conditions` (LightingConfig): Lighting at capture time
- `weather_conditions` (WeatherConditions): Weather simulation parameters
- `robot_pose` (PoseStamped): Robot position and orientation during capture

### 20. Transform
Representation of position and orientation transformation.

**Fields**:
- `translation` (Vector3): Translation vector (x, y, z)
- `rotation` (Quaternion): Rotation as quaternion (x, y, z, w)

## Relationships

1. **VirtualEnvironment** contains many **SimulatedObject**
2. **HumanoidRobotModel** contains many **Link**, **Joint**, and **SimulatedSensor**
3. **Joint** connects exactly two **Link** elements (parent and child)
4. **SimulatedSensor** is attached to one **Link** of a **HumanoidRobotModel**
5. **NavigationPlan** is associated with one **HumanoidRobotModel**
6. **NavigationPlan** contains many **TrajectoryPoint**
7. **SLAMMap** contains many **PoseStamped** and **Landmark**
8. **SyntheticDataset** contains many **DatasetFile**
9. **DatasetFile** may be associated with many **AnnotationFile**

## Validation Rules

1. Robot models must have at least 2 legs for bipedal locomotion
2. All transforms must result in physically plausible configurations
3. Navigation plans must avoid collisions with known obstacles in the SLAM map
4. Sensor data must include realistic noise models based on the hardware specifications
5. Kinematic chains must form valid configurations without self-collisions
6. SLAM maps must maintain temporal consistency in localization estimates