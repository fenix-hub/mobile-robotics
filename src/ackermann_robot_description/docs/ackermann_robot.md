# Ackermann Robot Overview

## What is an Ackermann Robot?
An Ackermann robot is a type of mobile robot that uses Ackermann steering geometry, which is commonly found in automobiles. Unlike differential drive robots (which steer by varying the speed of wheels on either side), Ackermann robots steer by turning their front wheels, similar to how a car operates. This steering mechanism allows for smoother turns and more realistic vehicle dynamics, making Ackermann robots ideal for applications that require car-like movement, such as autonomous driving research and urban navigation.

Ackermann steering ensures that all wheels follow concentric circles during a turn, minimizing tire slip and improving maneuverability. The key characteristics of an Ackermann robot include:
- **Front-wheel steering:** Only the front wheels change direction to steer the robot.
- **Rear-wheel drive:** The rear wheels provide the driving force.
- **Non-holonomic constraints:** The robot cannot move sideways, only forward and backward with steering.

![Screenshot from 2025-06-29 15-25-45.png](media/Screenshot%20from%202025-06-29%2015-25-45.png)


## Structure of Our Ackermann Robot (Based on the URDF)
Our Ackermann robot is modeled in URDF (Unified Robot Description Format) and consists of the following main components:

- **Chassis:** The main body of the robot, represented as a box. It serves as the base to which all other components are attached.
- **Wheels:** Four wheels are included:
  - **Front Wheels:** Mounted on steering links, allowing them to rotate for steering.
  - **Rear Wheels:** Fixed to the chassis and provide the driving force.
- **Steering Links and Joints:**
  - The front wheels are connected to the chassis via steering links and revolute joints, enabling them to turn left and right.
  - The rear wheels are attached with continuous joints, allowing them to rotate freely for propulsion.
- **Sensors:**
  - **LiDAR:** Mounted on top of the chassis for environment scanning.
  - **Depth Camera:** Positioned at the front for visual perception.
  - **IMU:** Placed at the center of the robot to measure orientation and acceleration.
- **ros2_control Integration:**
  - The robot uses the `ros2_control` framework for hardware interface, enabling control of wheel velocities and steering angles through defined command and state interfaces.
- **Gazebo Plugins:**
  - Plugins are included for simulating sensors and interfacing with Gazebo's physics engine.

### Key Parameters (from the URDF)
- **Chassis dimensions:** 0.6m (length) x 0.4m (width) x 0.1m (height)
- **Wheelbase:** 0.4m (distance between front and rear axles)
- **Track width:** 0.3m (distance between left and right wheels)
- **Wheel radius:** 0.05m

This structure closely mimics a real car, providing realistic steering and driving behavior in simulation. The URDF defines all physical properties, joint types, and sensor placements, ensuring the robot can be used effectively for navigation, mapping, and autonomous driving experiments in ROS 2 and Gazebo environments.
