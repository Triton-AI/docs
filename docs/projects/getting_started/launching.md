---
title: ROS2 Launch System
hide_title: True
description: A guide on how to use the ROS 2 launch system and run nodes.
sidebar_position: 3
---

## Running ROS2
In ROS2, there are **two main ways** to start your robot’s behavior:
1. Running individual **nodes** manually
2. Launching multiple nodes using a **launch file**

### Running a Node
This method is simple and useful for testing one component at a time. You are directly running the nodes instead of having a launcher launch the node for you.

#### Example
```bash
ros2 run <package_name> <node_executable>
ros2 run my_robot_sensors imu_reader
```

### Launching Nodes
For real deployments or simulations, you typically need multiple nodes to start together. You will also want the high level ability to configure your nodes. This is where launch files come in.

Every package has their own launch files:
```txt
my_package/
├── launch/
│   ├── sim.launch.py
│   └── robot.launch.py
├── src/
│   └── ...
├── CMakeLists.txt
└── package.xml
```
Launch files allow us to:
- Start multiple nodes together
- Set parameters for each node
- Remap topics
- Load configurations (YAML files, robot URDF, etc.)
- Automate simulation or real-world startup

:::info
It is recommended to keep one launch file per context (e.g. sim.launch.py, real.launch.py, rviz.launch.py
:::

#### Example
```bash
ros2 launch <package_name> <launch_file>.launch.py
```
