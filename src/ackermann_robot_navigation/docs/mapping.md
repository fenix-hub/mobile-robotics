# Mapping Process

This document describes the process of mapping using Gazebo, RViz2, SLAM Toolbox, Nav2 Bringup, and Teleop, utilizing the `obstacle.world` file.

![Screenshot from 2025-06-29 12-04-21.png](media/Screenshot%20from%202025-06-29%2012-04-21.png)

## Prerequisites
Ensure you have the following installed and configured:
- ROS 2 (Foxy or later)
- SLAM Toolbox
- Nav2 Bringup
- Teleop Twist Keyboard
- RViz2

## Steps

### 1. Launch the Simulation Environment
Start the simulation environment with the `obstacle.world` file:
```bash
ros2 launch ackermann_robot_description bringup_all.launch.py
```
This command initializes the robot description and loads the `obstacle.world` file in Gazebo.

### 2. Start SLAM Toolbox
Run the SLAM Toolbox in asynchronous mode with simulation time enabled:
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```
This enables the robot to perform SLAM (Simultaneous Localization and Mapping) in the simulated environment.

### 3. Launch Navigation Stack
Start the Nav2 navigation stack with simulation time enabled:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```
This initializes the navigation stack, including path planning and obstacle avoidance.

### 4. Control the Robot
Use the Teleop Twist Keyboard to manually control the robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
This allows you to drive the robot around the environment to explore and map it.

### 5. Visualize in RViz2
Open RViz2 with the default navigation configuration:
```bash
ros2 run rviz2 rviz2 -d /opt/ros/foxy/share/nav2_bringup/rviz/nav2_default_view.rviz
```
This provides a visual representation of the robot, the map being generated, and other navigation data.

![Screenshot from 2025-06-29 11-51-28.png](media/Screenshot%20from%202025-06-29%2011-51-28.png)
![Screenshot from 2025-06-29 11-52-59.png](media/Screenshot%20from%202025-06-29%2011-52-59.png)

### 6. Save the Map
Once the mapping process is complete, save the generated map:
```bash
ros2 run nav2_map_server map_saver_cli -f "ackermann_steering_map" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/
```
This saves the map to a file named `ackermann_steering_map` for future use.

## Notes
- Ensure all nodes are running in the same ROS 2 workspace.
- Use the `obstacle.world` file to simulate a realistic environment for mapping.
- Adjust parameters in the launch files as needed for your specific setup.
