---
sidebar_position: 2
title: 'Chapter 1: ROS 2 Fundamentals'
---

# Chapter 1: ROS 2 Fundamentals

import LearningObjective from '@site/src/components/LearningObjective';
import ImportantNote from '@site/src/components/ImportantNote';

<LearningObjective>
After completing this chapter, you will be able to:
- Explain the purpose and architecture of ROS 2
- Identify the core communication patterns: nodes, topics, services, and actions
- Understand when to use each communication pattern appropriately
- Describe the distributed system design of ROS 2
</LearningObjective>

## Introduction to ROS 2

Robot Operating System 2 (ROS 2) is not an operating system in the traditional sense, but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Why ROS 2?

ROS 2 was developed to address the limitations of the original ROS framework, particularly in areas of:

- **Real-time support**: Better real-time capabilities for time-critical applications
- **Multi-robot systems**: Improved support for multiple robots working together
- **Commercial product support**: More suitable for deployment in production environments
- **Security**: Built-in security features for safe operation
- **Architecture**: Improved middleware architecture with DDS (Data Distribution Service)

## The ROS 2 Architecture

ROS 2 uses a distributed architecture where multiple processes (potentially on different machines) communicate with each other. This communication is facilitated by the DDS (Data Distribution Service) middleware, which handles the underlying network communication.

### Key Concepts

- **Nodes**: Processes that perform computation. Nodes are the basic compute element of a ROS 2 program.
- **Packages**: Collections of nodes, libraries, and other resources that perform a specific function.
- **Topics**: Named buses over which nodes exchange messages.
- **Messages**: Data structures that are passed between nodes.
- **Services**: Synchronous request/reply communication between nodes.
- **Actions**: Asynchronous request/reply communication with feedback and goal preemption.

## Communication Patterns

ROS 2 provides several communication patterns for different use cases. Understanding when to use each pattern is crucial for effective robot programming.

### Nodes

A node is the fundamental unit of execution in ROS 2. It is a process that performs computation. Nodes are organized into packages to be built and distributed.

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

<ImportantNote>
In ROS 2, nodes must be explicitly initialized and cleaned up. The rclpy.init() function initializes the ROS 2 client library, and rclpy.shutdown() cleans it up.
</ImportantNote>

### Topics and Publishers/Subscribers

Topics are named buses over which nodes exchange messages. This is a **publish/subscribe** pattern where publishers send messages to a topic and subscribers receive messages from a topic.

- **Publishers** send data to a topic
- **Subscribers** receive data from a topic
- Communication is **asynchronous** and **one-way**
- Multiple publishers and subscribers can exist for the same topic

### Services

Services provide **request/reply** communication between nodes. This is a **synchronous** communication pattern where:

- A client sends a request to a service
- The service processes the request and sends back a response
- The client waits for the response (blocking)

```python
# Service request
request = AddTwoInts.Request()
request.a = 2
request.b = 3

# Send request and get response
future = client.call_async(request)
```

### Actions

Actions are for **long-running tasks** with the following characteristics:

- **Goal**: Request for a task to be performed
- **Feedback**: Periodic updates during task execution
- **Result**: Final outcome of the task
- Non-blocking: Client can cancel the action or check on its status

Actions are ideal for tasks like:
- Navigation to a goal location
- Object manipulation sequences
- Calibration procedures

## When to Use Each Communication Pattern

### Use Topics when:

- You need **asynchronous** communication
- Multiple nodes need to receive the same information simultaneously
- The sender doesn't need to know if anyone is listening
- You want to decouple the timing of publishers and subscribers

### Use Services when:

- You need **synchronous** request/reply communication
- You need a response before proceeding
- The task is relatively quick to complete
- You need to ensure the request was processed

### Use Actions when:

- You need to perform **long-running tasks**
- You want to provide feedback during execution
- You need the ability to cancel the task
- The task might take seconds, minutes, or even hours to complete

## Practical Example: Communication Patterns in a Mobile Robot

Consider a mobile robot with the following components:

1. **Navigation System**: Plans paths and sends movement commands
2. **Sensor Node**: Publishes laser scan data
3. **Movement Controller**: Executes movement commands
4. **Battery Monitor**: Reports battery status

This system might use:
- **Topics**: Sensor data (laser scans) published to multiple subscribers
- **Services**: Requesting a new path calculation from the navigation system
- **Actions**: Executing a complex navigation task with feedback on progress

## Summary

In this chapter, we've covered the fundamental concepts of ROS 2 architecture and communication patterns. Understanding these concepts is crucial for developing effective robotic applications. The choice of communication pattern (topics, services, or actions) depends on your specific use case and requirements.

In the next chapter, we'll dive deeper into implementing these communication patterns in code.

## Next Steps

Continue to [Chapter 2: Robot Control Programming](../chapter2-control) to learn how to implement these concepts in practice.

## Exercises

import Exercise from '@site/src/components/Exercise';

<Exercise title="Exercise 1: Communication Pattern Identification" difficulty="beginner">
For each of the following scenarios, identify which ROS 2 communication pattern (topic, service, or action) would be most appropriate and explain why:

1. A sensor node publishing laser scan data that multiple other nodes need to process
2. Requesting the robot to move to a specific location and waiting for confirmation
3. A complex task like mapping a large area that takes several minutes and requires progress updates
4. Broadcasting the robot's current position to all interested nodes
5. Requesting the current battery level and receiving an immediate response
</Exercise>

<Exercise title="Exercise 2: Architecture Design" difficulty="intermediate">
Design a simple robot system with three nodes that demonstrates all three communication patterns:
- One node that publishes sensor data (using topics)
- One node that provides a service for calculating distances
- One node that performs a long-running navigation task (using actions)

Describe how these nodes would interact and what messages they would exchange.
</Exercise>

## Further Reading

- [ROS 2 Documentation](https://docs.ros.org/en/rolling/)
- [ROS 2 Design](https://design.ros2.org/)
- [DDS (Data Distribution Service) Overview](https://www.dds-foundation.org/what-is-dds/)
- [ROS 2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html)