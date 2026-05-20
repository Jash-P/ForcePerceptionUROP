# ros2_ws

This workspace contains the following ROS 2 packages for UR5 + AR10 hand integration, simulation, and manipulation:

- `ur5_moveit_config`: MoveIt 2 configuration for the UR5 arm (with AR10 as end-effector)
- `ar10_hand`: ROS 2 node for controlling the AR10 hand via UDP to RP2040
- `grasp_manager`: Custom action server for coordinated grasping
- `collision_monitor`: Node for collision monitoring and safety override
- `ur5_bringup`: Launch files for bringing up the real or simulated system
- `ur_description`: URDF/Xacro for the UR5 arm
- `ar10_description`: URDF/Xacro for the AR10 hand
- `gazebo_sim`: Gazebo simulation launch and configuration
- `custom_msgs`: Custom message definitions (e.g., CollisionAlert)
