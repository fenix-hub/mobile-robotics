# SalviSantilio - Mobile Robotics Project

This repository contains a minimal setup for an Ackermann drive robot using the [nav2](https://navigation.ros.org/) stack on **ROS 2 Foxy**. It provides launch files and parameters to simulate the vehicle in Gazebo and autonomously navigate around a map.

![simplescreenrecorder-2025-06-29_15.02.40.gif](src/ackermann_robot_navigation/docs/media/simplescreenrecorder-2025-06-29_15.02.40.gif)

## Rationale
The project aims to demonstrate the capabilities of the ROS 2 navigation stack with an Ackermann steering model, which is commonly used in autonomous vehicles.  
At first, we considered using the [AWS DeepRacer](https://aws.amazon.com/deepracer/) platform, but it was not compatible with ROS 2 Foxy. Therefore, we created a custom Ackermann robot model and navigation stack that works seamlessly with the nav2 stack.  
Then, we tried to use the ackermann steering model from the official ROS2 Gazebo / Nav2 stack, [but there were no out-of-the-box solutions for Ackermann steering in ROS 2 Foxy](https://robostack.github.io/foxy.html).  
Thus, we implemented our own custom odometry publisher and control bridge to handle Ackermann steering kinematics, allowing the robot to navigate effectively in simulated environments.  

## Repository Structure
- `src/ackermann_robot_description` – URDF model and Gazebo launch files for the robot
- `src/ackermann_control_bridge` – node that converts `/cmd_vel` into steering and wheel commands
- `src/ackermann_robot_navigation` – navigation configuration and launch files
- The robot description now includes a 2D LIDAR publishing `/scan` and a depth camera publishing `/depth/image_raw` in Gazebo.

## Documentation
You can find detailed documentation for each package in the `docs` directory:
- [ackermann_robot_description](src/ackermann_robot_description/docs/ackermann_robot.md) – Overview of the Ackermann robot structure and URDF
- [ackermann_bridge](src/ackermann_control_bridge/docs/ackermann_control_bridge.md) – Overview of the control bridge node and its functionality
- [ackermann_control_bridge](src/ackermann_control_bridge/docs/odometry.md) – Custom odometry publisher for Ackermann steering
- [ackermann_robot_navigation](src/ackermann_robot_navigation/docs/mapping.md) – Instructions for mapping and navigation setup

## How the nav2 Stack Works
The nav2 stack is a collection of ROS 2 nodes that provide navigation capabilities:

1. **map_server** – loads an occupancy grid map for global planning.
2. **amcl** – localizes the robot using Adaptive Monte Carlo Localization.
3. **planner_server** – computes paths from the current pose to a goal pose.
4. **controller_server** – follows the global plan by generating velocity commands.
5. **bt_navigator** – runs a Behavior Tree (BT) that sequences high‑level navigation behaviors.
6. **recoveries_server** – executes recovery behaviors when navigation fails.
7. **nav2_lifecycle_manager** – manages node startup and shutdown.

The behavior tree used here is `navigate_w_replanning_and_recovery.xml`, which repeatedly replans the path at 1 Hz and performs clearing, spinning, waiting, and backing up if issues occur.

## Bringing Up the Navigation Stack
The entire navigation stack, along with the robot simulation and visualization, can be launched using the `bringup_all.launch.py` file:

```bash
ros2 launch ackermann_robot_navigation bringup_all.launch.py
```

This will:
- Launch the Gazebo simulation with the Ackermann robot.
- Start the control bridge and odometry nodes.
- Bring up the nav2 stack (map server, AMCL, planner, controller, etc.).
- Open RViz2 for visualization.

## Quickstart: Install System Dependencies

Before building the workspace, install all required apt packages:

```bash
bash install_ros2_foxy_dependencies.sh
```

## Building the Workspace
From the repository root:
```bash
source /opt/ros/foxy/setup.bash   # make sure ROS 2 Foxy is installed
colcon build --symlink-install
```

## Running the Simulation and nav2
1. **Start Gazebo and the robot**
   ```bash
   ros2 launch ackermann_robot_navigation bringup_all.launch.py
   ```
2. **Start the nav2 stack** (in another terminal):
   ```bash
   ros2 launch ackermann_robot_navigation nav2_bringup.launch.py
   ```

The robot will read the map in `src/ackermann_robot_navigation/maps/ackermann_steering_map.yaml` and you can send goals using RViz2 or the `/navigate_to_pose` action.

The LiDAR and depth camera data is available on `/scan` and `/depth/image_raw`.

## Sending Navigation Goals

You can send navigation goals using RViz2 or directly via command line:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```
3. **Visualize in RViz2** (in another terminal):
   ```bash
   ros2 run rviz2 rviz2 -d src/ackermann_robot_navigation/rviz/nav2_default_view.rviz
   ```
4. **Send goals** using the RViz2 interface or command line as shown above.
5. **Monitor the robot's state** in RViz2, where you can see the robot's position, planned path, and sensor data.

---

## Bibliography and References

1. [Foxy support on Ackermann](https://robostack.github.io/foxy.html)
2. [AWS Deepracer Navigation Stack](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md)
3. [ROS 2 Navigation Documentation](https://docs.nav2.org/)
4. [ROS2 Nav2 Foxy](https://github.com/ros-navigation/navigation2/tree/foxy-devel)