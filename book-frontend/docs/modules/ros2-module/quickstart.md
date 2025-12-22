---
sidebar_position: 5
title: 'Quick Start Guide'
---

# Quick Start Guide: ROS 2 Robotics Module

This quick start guide provides a condensed overview of the ROS 2 Robotics Module, designed for students who want to quickly understand the key concepts and get hands-on with ROS 2.

## Overview

This module teaches you how to work with ROS 2 (Robot Operating System 2), a flexible framework for writing robot software. You'll learn about:

- Core ROS 2 concepts and architecture
- Programming robot control systems
- Creating robot descriptions and models

## Prerequisites

Before starting this module, you should have:
- Basic Python programming knowledge
- Understanding of fundamental programming concepts (variables, functions, classes)
- Familiarity with command line interfaces

## Chapter Summary

### Chapter 1: ROS 2 Fundamentals
- **Core Concepts**: Nodes, topics, services, and actions
- **Architecture**: Distributed system design with DDS middleware
- **Communication Patterns**: When to use each pattern

**Quick Example - Publisher Node**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = MinimalPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
```

### Chapter 2: Robot Control Programming
- **Programming with rclpy**: Python client library for ROS 2
- **Publishers and Subscribers**: Asynchronous communication
- **Services**: Synchronous request/reply communication
- **Actions**: Long-running tasks with feedback

### Chapter 3: Robot Description and Modeling
- **URDF**: Unified Robot Description Format
- **Links and Joints**: Defining robot structure
- **Visualization**: Tools for viewing robot models

## Getting Started Steps

1. **Install ROS 2**: Follow the official installation guide for your operating system
2. **Set up your workspace**: Create a colcon workspace for your projects
3. **Run the examples**: Execute the example code from each chapter
4. **Complete the exercises**: Practice with the hands-on exercises
5. **Build your own robot description**: Apply concepts to create your own robot model

## Common Commands

```bash
# Source ROS 2 environment
source /opt/ros/rolling/setup.bash

# Create a new package
ros2 pkg create --build-type ament_python my_robot_package

# Build your workspace
colcon build

# Run a node
ros2 run my_package my_node

# List active topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /my_topic std_msgs/msg/String
```

## Troubleshooting

- **Node not found**: Make sure you've sourced your workspace (`source install/setup.bash`)
- **Topic not connecting**: Check that the publisher and subscriber use the same topic name and message type
- **Import errors**: Ensure rclpy is installed (`pip install rclpy` or install ROS 2)

## Next Steps

After completing this quick start guide, proceed with the full module to gain deeper understanding:

1. Start with [Chapter 1: ROS 2 Fundamentals](./chapter1-fundamentals) for theoretical foundations
2. Move to [Chapter 2: Robot Control Programming](./chapter2-control) for practical implementation
3. Finish with [Chapter 3: Robot Description and Modeling](./chapter3-urdf) for robot modeling

## Resources

- [Official ROS 2 Documentation](https://docs.ros.org/en/rolling/)
- [ROS 2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html)
- [ROS Answers](https://answers.ros.org/questions/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)