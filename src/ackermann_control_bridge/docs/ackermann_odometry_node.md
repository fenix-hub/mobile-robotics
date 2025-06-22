# Ackermann Odometry Node Documentation

## Overview

The `ackermann_odometry_node.py` is a ROS 2 node designed to provide odometry information for an Ackermann steering robot. This node is essential for integration with RViz visualization and Nav2 navigation stack, which rely on accurate odometry data. The implementation follows REP-105 (ROS Enhancement Proposal) standards for coordinate frames and transformations.

## REP-105 Compliance

This odometry system follows the ROS REP-105 standard for coordinate frames:

- **base_link**: A frame rigidly attached to the mobile robot base at its 3D center
- **base_footprint**: A frame representing the robot's projection on the ground
- **odom**: A world-fixed frame that provides locally accurate continuous position estimates
- **map**: A world-fixed frame that can have discrete jumps (handled by localization systems)

The odometry node specifically handles the odom → base_footprint transformation, which is a critical component of the navigation stack's transform hierarchy:

```
map → odom → base_footprint → base_link
```

## Features

- Publishes odometry data on the `/odom` topic using the `nav_msgs/Odometry` message type
- Broadcasts the TF transformation from `odom` to `base_footprint` frames
- Multi-sensor fusion from up to four sources:
  - Wheel encoders for linear velocity and basic angular velocity
  - IMU for accurate orientation and angular velocity
  - LiDAR scan matching for improved position estimation and drift correction
  - Camera-based visual odometry for additional motion detection
- Configurable odom frame positioning to place it at a distance from the robot (similar to AWS DeepRacer)
- Support for initial pose updates from localization systems
- Adaptive weighting system for sensor fusion
- Proper covariance estimation for downstream consumers
- Handles NaN values in sensor data

## Prerequisites

- ROS 2 Foxy or newer
- Ackermann steering robot model with:
  - Wheel velocity controllers
  - Steering angle controllers
  - IMU sensor publishing to `/imu` topic
  - LiDAR publishing to `/scan` topic
  - Camera publishing to `/depth_camera/image_raw` topic
  - (Optional but recommended) Wheel encoders for velocity feedback
- Additional dependencies:
  - `cv_bridge` for camera image processing
  - OpenCV for visual odometry

## Usage

### Launch the Node

```bash
ros2 launch ackermann_control_bridge odometry.launch.py
```

### Launch Arguments

You can customize the odometry node behavior by passing arguments to the launch file:

```bash
ros2 launch ackermann_control_bridge odometry.launch.py use_imu:=true use_lidar:=true use_camera:=false odom_position_offset_x:=10.0 odom_position_offset_y:=10.0
```

### Parameters

The node can be configured with the following parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheelbase` | 0.4 | Distance between front and rear axles (meters) |
| `track_width` | 0.3 | Distance between left and right wheels (meters) |
| `wheel_radius` | 0.05 | Radius of the wheels (meters) |
| `odom_frame` | "odom" | Name of the odometry frame |
| `base_frame` | "base_link" | Name of the robot's base frame |
| `footprint_frame` | "base_footprint" | Name of the robot's footprint frame |
| `publish_rate` | 50.0 | Rate at which odometry is published (Hz) |
| `use_sim_time` | true | Whether to use simulation time |
| `use_joint_states` | true | Whether to use joint_states topic for feedback |
| `use_imu` | true | Whether to use IMU data for orientation and angular velocity |
| `use_lidar` | true | Whether to use LiDAR data for scan matching odometry |
| `use_camera` | false | Whether to use camera data for visual odometry |
| `imu_orientation_weight` | 0.7 | Weight for IMU orientation in sensor fusion (0-1) |
| `wheel_encoder_weight` | 0.7 | Weight for wheel encoder data in sensor fusion (0-1) |
| `lidar_weight` | 0.5 | Weight for LiDAR scan matching in sensor fusion (0-1) |
| `camera_weight` | 0.3 | Weight for visual odometry in sensor fusion (0-1) |
| `odom_position_offset_x` | 10.0 | X offset for the odom frame (meters) |
| `odom_position_offset_y` | 10.0 | Y offset for the odom frame (meters) |
| `initial_pose_x` | 0.0 | Initial robot X position |
| `initial_pose_y` | 0.0 | Initial robot Y position |
| `initial_pose_yaw` | 0.0 | Initial robot yaw orientation |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for the robot |
| `/joint_states` | `sensor_msgs/JointState` | Joint position and velocity information |
| `/imu` | `sensor_msgs/Imu` | IMU data for orientation and angular velocity |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data for scan matching |
| `/depth_camera/image_raw` | `sensor_msgs/Image` | Camera images for visual odometry |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Initial pose updates from RViz or localization systems |
| `/rear_left_wheel_velocity_controller/velocity` | `std_msgs/Float64` | Left rear wheel velocity feedback (if not using joint_states) |
| `/rear_right_wheel_velocity_controller/velocity` | `std_msgs/Float64` | Right rear wheel velocity feedback (if not using joint_states) |
| `/front_left_wheel_position_controller/position` | `std_msgs/Float64` | Left front wheel steering angle (if not using joint_states) |
| `/front_right_wheel_position_controller/position` | `std_msgs/Float64` | Right front wheel steering angle (if not using joint_states) |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Odometry information with position, orientation, and velocity |

## TF Transforms

- `odom` → `base_footprint`: Transformation representing the robot's position and orientation in the odometry frame

## Frame Hierarchy and Coordinate Systems

In accordance with REP-105, the coordinate systems follow these conventions:

1. The **odom** frame:
   - Provides continuous and locally accurate position estimation
   - Origin remains at a fixed offset from the robot (configurable via parameters)
   - Z-axis points upward
   - Used by the navigation stack for local planning

2. The **base_footprint** frame:
   - Represents the projection of the robot onto the ground plane
   - Serves as the reference for the robot's position in the world
   - Is the direct child of the odom frame

3. The **base_link** frame:
   - Rigidly attached to the robot's body at its 3D center
   - Defined in the URDF file with proper relation to base_footprint

4. The **map** frame (provided by SLAM or localization systems):
   - Global reference frame for localization
   - Can have discrete jumps as the localization refines
   - Z-axis points upward

The full transformation tree follows:
```
map → odom → base_footprint → base_link → [sensor frames]
```

## Multi-Sensor Fusion Approach

The node implements a sophisticated weighted averaging approach for sensor fusion from four possible sources:

1. **Wheel Encoders**:
   - Provide basic odometry through wheel velocities and steering angles
   - Primary source for forward/backward motion
   - Most reliable on smooth surfaces with good traction
   - Weighted by `wheel_encoder_weight` parameter

2. **IMU**:
   - Provides high-frequency orientation and angular velocity
   - Excellent for short-term accuracy in rotation estimation
   - Compensates for quick movements that wheel encoders might miss
   - Weighted by `imu_orientation_weight` parameter

3. **LiDAR Scan Matching**:
   - Compares consecutive LiDAR scans to estimate robot motion
   - Can detect lateral movement not measurable by wheel encoders
   - Particularly useful for correcting drift in wheel odometry
   - More accurate in feature-rich environments
   - Weighted by `lidar_weight` parameter

4. **Visual Odometry (Camera)**:
   - Uses feature tracking between consecutive camera frames
   - Provides an additional motion estimate independent of wheel slip
   - Can work in environments where LiDAR might struggle (highly reflective surfaces)
   - Computationally intensive but offers valuable redundancy
   - Weighted by `camera_weight` parameter

The fusion algorithm dynamically combines these sources based on:
- Availability of each sensor
- Configured weights
- Consistency checks between different sensors

In scenarios where sensor data conflicts, the system prioritizes based on the weights while maintaining a smooth trajectory estimate.

## Scan Matching and Visual Odometry Approaches

### LiDAR Scan Matching

The node uses a simplified scan matching approach that:
1. Stores consecutive LiDAR scans
2. Compares scan points to estimate relative motion
3. Converts this motion to velocity components for fusion

Note: The current implementation contains a placeholder for scan matching. For production use, consider implementing a more sophisticated algorithm like ICP (Iterative Closest Point) or correlative scan matching.

### Visual Odometry

The visual odometry component:
1. Processes incoming camera images using OpenCV via cv_bridge
2. Tracks features between consecutive frames
3. Estimates camera motion from feature displacement
4. Converts this to robot motion in the odometry frame

Note: The current implementation contains a placeholder for visual odometry. For production use, consider implementing a feature-based approach like ORB-SLAM or a direct method like DSO.

## Integration with Navigation Stack

For proper integration with Nav2:

1. This node provides the `/odom` topic and the odom → base_footprint transform
2. A localization system (like AMCL) should provide the map → odom transform
3. The URDF defines the base_footprint → base_link transform
4. Sensor transforms are typically defined in the URDF

This separation of responsibilities follows the standard pattern for ROS-based mobile robots and ensures proper operation of the navigation stack.

## Limitations and Considerations

- **Odometry drift**: Even with multi-sensor fusion, some drift is inevitable. This will be corrected by global localization.
- **Computational requirements**: Visual odometry and scan matching can be computationally intensive. Disable if resources are limited.
- **Environment dependency**: LiDAR scan matching works best in feature-rich environments, while visual odometry needs well-lit scenes with texture.
- **Wheel slip**: While sensor fusion mitigates wheel slip issues, extreme cases can still cause errors.
- **IMU calibration**: For best results, ensure your IMU is properly calibrated.
- **Frame offsets**: The odom frame is positioned with an offset from the robot, which helps visualization but does not affect functionality.

## Troubleshooting

- **Jumpy odometry**: Check the IMU data quality and possibly reduce the `imu_orientation_weight`
- **Slow or delayed odometry**: If using camera or LiDAR processing, consider reducing their weights or disabling them entirely
- **Poor turning accuracy**: Ensure the `wheelbase` and `track_width` parameters match your robot's physical dimensions
- **TF errors**: Check that the frame names in parameters match your robot's URDF
- **Missing sensor data**: Verify that all sensors are publishing at the expected rates
- **OpenCV errors**: Make sure cv_bridge is properly installed if using camera-based odometry
