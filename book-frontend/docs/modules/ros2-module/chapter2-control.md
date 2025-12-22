---
sidebar_position: 3
title: 'Chapter 2: Robot Control Programming'
---

# Chapter 2: Robot Control Programming

import LearningObjective from '@site/src/components/LearningObjective';
import ImportantNote from '@site/src/components/ImportantNote';
import Exercise from '@site/src/components/Exercise';

<LearningObjective>
After completing this chapter, you will be able to:
- Create publishers and subscribers to exchange robot communication topics
- Implement services for synchronous communication between nodes
- Bridge AI agents to robot controllers using appropriate programming interfaces
- Use rclpy for Python-based robot control programming
- Understand the practical aspects of connecting AI logic to physical robot actions
</LearningObjective>

## Introduction to Robot Control Programming

In the previous chapter, we learned about the theoretical concepts of ROS 2 communication patterns. In this chapter, we'll dive into the practical implementation of these concepts using the Python client library for ROS 2 (rclpy).

Robot control programming involves creating nodes that can communicate with other nodes in the ROS 2 ecosystem. This includes:

- **Publishers**: Nodes that send data to topics
- **Subscribers**: Nodes that receive data from topics
- **Services**: Nodes that provide synchronous request/reply communication
- **Actions**: Nodes that handle long-running tasks with feedback

## Programming with rclpy

The Python client library for ROS 2 (rclpy) provides the tools needed to create ROS 2 nodes in Python. It's designed to be intuitive for Python developers while providing access to all of ROS 2's powerful features.

### Setting Up a Basic Node

Before creating any ROS 2 functionality, we need to initialize the ROS 2 client library:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create a node
    minimal_node = MyNode()

    # Spin the node to process callbacks
    rclpy.spin(minimal_node)

    # Destroy the node explicitly (optional but recommended)
    minimal_node.destroy_node()

    # Shutdown the ROS 2 client library
    rclpy.shutdown()

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code goes here
```

<ImportantNote>
The rclpy.init() function must be called before creating any nodes, and rclpy.shutdown() should be called when you're done to properly clean up resources.
</ImportantNote>

## Publishers

Publishers send messages to topics. Here's how to create a publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer to publish messages periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher Best Practices

- Always specify an appropriate queue size for your publisher (the third parameter in create_publisher)
- Use appropriate message types from standard ROS 2 message packages when possible
- Log messages appropriately to help with debugging

## Subscribers

Subscribers receive messages from topics. Here's how to create a subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)  # queue size
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Best Practices

- Choose an appropriate queue size based on how frequently messages arrive and how quickly your callback can process them
- Keep subscriber callbacks fast to avoid blocking message processing
- Handle different message types appropriately

## Services

Services provide synchronous request/reply communication. Here's how to create both a service server and client:

### Service Server

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')

        # Create a service
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python
import rclpy
from rclpy.node import Node
from add_two_ints_srv.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')

        # Create a client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result of add_two_ints: %d' % response.sum)

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions

Actions are used for long-running tasks. Here's how to implement them:

### Action Server

```python
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info('Publishing feedback: {0}'.format(
                feedback_msg.sequence))

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Returning result: {0}'.format(result.sequence))

        return result
```

### Action Client

```python
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(
            feedback.sequence))

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
```

## Bridging AI Agents to Robot Controllers

One of the key concepts in robotics is bridging AI agents to robot controllers. This involves:

1. **AI Logic Layer**: High-level decision making (path planning, task planning, etc.)
2. **Bridge Layer**: Translates AI decisions to robot commands
3. **Robot Controller Layer**: Low-level control of robot hardware

### Example: AI-Driven Navigation

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class AINavBridge(Node):
    def __init__(self):
        super().__init__('ai_nav_bridge')

        # Publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for AI commands
        self.ai_command_subscriber = self.create_subscription(
            String,
            '/ai_commands',
            self.ai_command_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.current_command = None
        self.velocity = Twist()

    def ai_command_callback(self, msg):
        """Process commands from AI agent"""
        command = msg.data
        self.get_logger().info(f'AI Command received: {command}')

        # Parse AI command and convert to robot control
        if command == 'move_forward':
            self.current_command = 'forward'
        elif command == 'turn_left':
            self.current_command = 'left'
        elif command == 'turn_right':
            self.current_command = 'right'
        elif command == 'stop':
            self.current_command = 'stop'

    def control_loop(self):
        """Main control loop"""
        if self.current_command == 'forward':
            self.velocity.linear.x = 0.5  # Move forward at 0.5 m/s
            self.velocity.angular.z = 0.0
        elif self.current_command == 'left':
            self.velocity.linear.x = 0.2  # Move forward while turning
            self.velocity.angular.z = 0.5  # Turn left at 0.5 rad/s
        elif self.current_command == 'right':
            self.velocity.linear.x = 0.2  # Move forward while turning
            self.velocity.angular.z = -0.5  # Turn right at 0.5 rad/s
        elif self.current_command == 'stop':
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
        else:
            # Default: stop
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0

        # Publish velocity command
        self.cmd_vel_publisher.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    ai_nav_bridge = AINavBridge()
    rclpy.spin(ai_nav_bridge)
    ai_nav_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this chapter, we've covered the practical implementation of ROS 2 communication patterns using Python. We've learned how to create publishers, subscribers, services, and actions, and how to bridge AI agents to robot controllers. These skills are essential for connecting AI logic to physical robot actions.

## Next Steps

Previous: [Chapter 1: ROS 2 Fundamentals](../chapter1-fundamentals)
Next: [Chapter 3: Robot Description and Modeling](../chapter3-urdf)

## Exercises

<Exercise title="Exercise 1: Publisher-Subscriber Implementation" difficulty="beginner">
Create a simple publisher that publishes a counter value every second and a subscriber that prints the received value to the console. Make sure to:

1. Use appropriate message types (Int32 or similar)
2. Include proper node initialization and cleanup
3. Add logging to track published and received messages
</Exercise>

<Exercise title="Exercise 2: Service Implementation" difficulty="intermediate">
Implement a service that takes two coordinates (x, y) and returns the Euclidean distance from the origin (0, 0). Create both the service server and a client that calls the service with different coordinates.

1. Define a custom service message for the request/response
2. Implement the server that calculates the distance
3. Implement a client that tests the service with multiple coordinate pairs
</Exercise>

<Exercise title="Exercise 3: AI-Controller Bridge" difficulty="advanced">
Design and implement a bridge node that connects an AI agent to a robot controller. The AI agent sends high-level commands (e.g., "go to location X,Y", "pick up object", "avoid obstacle") and the bridge translates these to low-level robot commands (velocity, joint angles, etc.).

1. Define appropriate message types for AI commands
2. Implement the bridge node with proper state management
3. Include safety checks to prevent invalid robot commands
</Exercise>

## Further Reading

- [rclpy API Documentation](https://docs.ros.org/en/rolling/p/rclpy/)
- [ROS 2 Client Libraries](https://docs.ros.org/en/rolling/Concepts/About-Client-Libraries.html)
- [ROS 2 Message Types](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html)
- [ROS 2 Actions Design](https://design.ros2.org/articles/actions.html)
- [Python Robotics Stack](https://github.com/ros2/rclpy)