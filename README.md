# Mobile Robotics Navigation

This repository contains a minimal setup for an Ackermann drive robot using the [nav2](https://navigation.ros.org/) stack on **ROS 2 Foxy**.  It provides launch files and parameters to simulate the vehicle in Gazebo and autonomously navigate around a map.

## Repository Structure
- `src/ackermann_robot_description` – URDF model and Gazebo launch files for the robot
- `src/ackermann_control_bridge` – node that converts `/cmd_vel` into steering and wheel commands
- `src/ackermann_robot_navigation` – navigation configuration and launch files
- The robot description now includes a 2D LIDAR publishing `/scan` and a depth camera publishing `/depth/image_raw` in Gazebo.

## How the nav2 Stack Works
The nav2 stack is a collection of ROS 2 nodes that provide navigation capabilities:

1. **map_server** – loads an occupancy grid map for global planning.
2. **amcl** – localizes the robot using Adaptive Monte Carlo Localization.
3. **planner_server** – computes paths from the current pose to a goal pose.
4. **controller_server** – follows the global plan by generating velocity commands.
5. **bt_navigator** – runs a Behavior Tree (BT) that sequences high‑level navigation behaviors.
6. **recoveries_server** – executes recovery behaviors when navigation fails.
7. **nav2_lifecycle_manager** – manages node startup and shutdown.

The behavior tree used here is `navigate_w_replanning_and_recovery.xml`, which repeatedly replans the path at 1 Hz and performs clearing, spinning, waiting and backing up if issues occur.

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

The robot will read the map in `src/ackermann_robot_navigation/maps/blank_map.yaml` and you can send goals using RViz2 or the `/navigate_to_pose` action.

The LiDAR and depth camera data is available on `/scan` and `/depth/image_raw`.
