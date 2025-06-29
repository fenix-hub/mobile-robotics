# Ackermann Control Bridge

## What is Ackermann Steering?
Ackermann steering is a steering geometry used in vehicles with front-wheel steering, such as cars. It ensures that during a turn, all wheels follow concentric circular paths, minimizing tire slip and allowing for smooth, realistic vehicle motion. In Ackermann steering, only the front wheels pivot to steer, while the rear wheels follow, constrained to move forward or backward without lateral slip. This is different from differential drive robots, which steer by varying the speed of wheels on each side.

![Ackerman+Steer+d+=+tan+=+o+d+d+o+i.jpg](media/Ackerman%2BSteer%2Bd%2B%3D%2Btan%2B%3D%2Bo%2Bd%2Bd%2Bo%2Bi.jpg)

## What is the `cmd_vel` Command?
In ROS (Robot Operating System), the `cmd_vel` topic is a standard interface for commanding robot motion. It accepts messages of type `geometry_msgs/Twist`, which specify the desired linear and angular velocities of the robot. For most robots, including differential and holonomic drives, this is the primary way to send movement commands from teleoperation tools, navigation stacks, or custom scripts.

## Driving an Ackermann Robot Using `cmd_vel`
Although the `cmd_vel` interface is generic, Ackermann steering robots require special handling to convert these velocity commands into appropriate steering angles and wheel velocities. In ROS2 Foxy and Gazebo Classic, there are no standard plugins or nodes that perform this conversion for Ackermann vehicles.

To address this, we implemented our own Ackermann steering algorithm, provided in the `ackermann_bridge_node.py` script. This algorithm:
- Listens to the `cmd_vel` topic for incoming velocity commands.
- Uses the Ackermann steering equations to compute the required steering angle for the front wheels and the velocity for the rear wheels, based on the commanded linear and angular velocities.
- Publishes the computed steering and velocity commands to the robot's actuators, enabling realistic Ackermann motion in simulation.

## How the Ackermann Bridge Node Works
The `ackermann_bridge_node.py` script is a ROS2 node that acts as a bridge between the standard `cmd_vel` interface and the low-level controllers of an Ackermann steering robot. Its main features are:

- **Subscriptions and Publications:**
  - Subscribes to `/cmd_vel` for velocity commands.
  - Publishes to the rear wheel velocity controllers and front wheel steering position controllers.
- **Parameter Handling:**
  - Uses parameters such as wheelbase, track width, and wheel radius, which should match the robot's URDF.
- **Ackermann Kinematics:**
  - Converts linear and angular velocity commands into steering angles for the front wheels and velocity commands for the rear wheels using Ackermann geometry.
  - Handles straight motion, turning, and even pure rotation (with limitations, as pure rotation is not typical for Ackermann vehicles).
- **Friction and Turning Compensation:**
  - Applies friction compensation and turning balance factors to improve simulation realism and reduce wheel slip.
  - Implements a minimum linear velocity threshold to ensure the robot moves smoothly from a stop.
- **Safety and Limits:**
  - Limits steering angles to physical constraints (±45°).
  - Applies deadzones to avoid minor unwanted rotations.
- **Debug Logging:**
  - Provides detailed logging for debugging and tuning.

## Teleoperation with Standard Tools
Thanks to this bridge, you can control the Ackermann robot using standard ROS teleoperation tools, such as `teleop_twist_keyboard`:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
This tool publishes `cmd_vel` messages as you press the arrow keys. Our Ackermann control bridge receives these commands, applies the Ackermann steering transformation, and drives the robot accordingly. This makes it possible to drive the robot as a car-like vehicle using only the standard ROS interfaces, even in environments where no native Ackermann support exists.

In summary, our custom Ackermann control bridge enables seamless control of an Ackermann steering robot in ROS2 Foxy and Gazebo Classic using the standard `cmd_vel` interface, filling a key gap in the available open-source tools. The node is robust, configurable, and designed to closely mimic real Ackermann vehicle behavior in simulation.
