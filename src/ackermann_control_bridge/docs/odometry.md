# Odometry in Ackermann Robot Navigation

## What is Odometry?
Odometry is the process of estimating a robot's position and orientation (pose) over time by integrating information from motion sensors, such as wheel encoders and inertial measurement units (IMUs). In mobile robotics, odometry provides the robot with a continuous estimate of its location relative to a starting point.

![Screenshot from 2025-06-29 15-22-25.png](media/Screenshot%20from%202025-06-29%2015-22-25.png)

### Odometry in SLAM, AMCL, and Nav2
- **SLAM (Simultaneous Localization and Mapping):** Odometry is used to provide an initial guess of the robot's movement between sensor updates, helping to build and update the map incrementally.
- **AMCL (Adaptive Monte Carlo Localization):** AMCL uses odometry as a motion model to predict the robot's pose between sensor observations, improving localization accuracy.
- **Nav2 Stack:** The navigation stack relies on odometry to estimate the robot's current pose for path planning, obstacle avoidance, and feedback control.

## Why Write a Custom Odometry Publisher?
For this project, we needed to implement our own odometry publisher because there is no out-of-the-box odometry node available for Ackermann steering vehicles in ROS 2 Foxy. Most existing libraries and packages are designed for differential drive robots and are not compatible with Ackermann kinematics, especially in Foxy, where support for custom vehicle models is limited.  
A more detailed explaination of the node can be found in the [ackermann_odometry_node.md](ackermann_odometry_node.md) documentation file.


## How the Custom Odometry Node Works
The custom odometry node (`ackermann_odometry_node.py`) was developed to address this gap. It performs the following functions:

- **Parameter Handling:** Loads robot-specific parameters such as wheelbase, track width, wheel radius, and joint names.
- **Sensor Fusion:** Subscribes to `/joint_states` (for wheel velocities and steering angles), `/imu/data` (for orientation), and `/cmd_vel` (for commanded velocities).
- **Ackermann Kinematics:** Calculates the robot's linear and angular velocities using the Ackermann steering model, which is essential for vehicles with front-wheel steering.
- **Sensor Fusion Weights:** Fuses data from wheel encoders, IMU, and command velocities using configurable weights to improve pose estimation robustness.
- **Pose Integration:** Integrates velocities over time to update the robot's pose (x, y, theta).
- **TF and Odometry Publishing:** Publishes the robot's pose as a transform (TF) and as a standard `nav_msgs/Odometry` message, including covariance estimates.

This approach ensures that the odometry is accurate and tailored to the specific kinematics of an Ackermann steering vehicle, which is critical for reliable navigation and mapping.

## Possible Improvements
- **Advanced Sensor Fusion:** Integrate an Extended Kalman Filter (EKF) or other state estimation techniques to better fuse IMU, encoder, and possibly GPS data.
- **Error Handling:** Improve robustness against sensor dropouts or faulty readings.
- **Dynamic Parameter Tuning:** Allow for dynamic reconfiguration of fusion weights and parameters at runtime.
- **Support for More Vehicle Types:** Extend the node to support other non-differential drive vehicles or more complex Ackermann models.

By developing this custom odometry node, the project enables reliable navigation and mapping for Ackermann steering robots in ROS 2 Foxy, filling a key gap in the available open-source ecosystem.
