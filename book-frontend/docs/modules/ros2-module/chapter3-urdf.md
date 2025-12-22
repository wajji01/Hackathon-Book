---
sidebar_position: 4
title: 'Chapter 3: Robot Description and Modeling'
---

# Chapter 3: Robot Description and Modeling

import LearningObjective from '@site/src/components/LearningObjective';
import ImportantNote from '@site/src/components/ImportantNote';
import Exercise from '@site/src/components/Exercise';

<LearningObjective>
After completing this chapter, you will be able to:
- Understand the Unified Robot Description Format (URDF) for robot modeling
- Define robot links, joints, and sensors in URDF files
- Visualize robot models using appropriate tools
- Create basic robot descriptions for humanoid robots
- Understand the relationship between robot description and simulation
</LearningObjective>

## Introduction to Robot Description Formats

Robot description is a crucial aspect of robotics that defines the physical and kinematic properties of a robot. The Unified Robot Description Format (URDF) is the standard format used in ROS for representing robot models. It's an XML-based format that describes robot links, joints, sensors, and other properties.

### Why Robot Description?

Robot description formats serve several important purposes:

1. **Simulation**: Describe the robot for physics simulation environments
2. **Visualization**: Define how the robot appears in visualization tools
3. **Kinematics**: Provide information for forward and inverse kinematics calculations
4. **Collision Detection**: Define collision geometry for safety and planning
5. **Control**: Supply necessary information for robot controllers

## Understanding URDF (Unified Robot Description Format)

URDF is an XML format that describes robot models. A URDF file contains:

- **Links**: Rigid bodies of the robot (e.g., chassis, arms, wheels)
- **Joints**: Connections between links that allow relative motion
- **Materials**: Visual appearance properties
- **Sensors**: Sensor definitions and mounting points
- **Gazebo plugins**: Simulation-specific properties

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Define links -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Define joints -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.2 0" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Links: The Building Blocks of Robots

Links represent rigid bodies in a robot. Each link can have:

- **Visual**: How the link appears in visualization
- **Collision**: Geometry used for collision detection
- **Inertial**: Mass, center of mass, and inertia properties

### Link Properties

```xml
<link name="link_name">
  <!-- Visual properties -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Can be box, cylinder, sphere, or mesh -->
      <box size="1 1 1"/>
    </geometry>
    <material name="color">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>

  <!-- Collision properties -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>

  <!-- Inertial properties -->
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

<ImportantNote>
Every URDF must have a single "base_link" or "base_footprint" as the root of the kinematic tree. All other links must be connected through joints.
</ImportantNote>

## Joints: Connecting the Links

Joints define how links can move relative to each other. There are several joint types:

- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement between links
- **floating**: 6 DOF movement (rarely used)
- **planar**: Movement in a plane

### Joint Properties

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Sensors in Robot Descriptions

Sensors can be attached to links in URDF files. While basic sensor definitions are possible in URDF, more complex sensor properties are typically handled in simulation environments like Gazebo.

```xml
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
</joint>

<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>

  <!-- Gazebo-specific sensor definition -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
    </sensor>
  </gazebo>
</link>
```

## Humanoid Robot Structure Basics

Humanoid robots have a specific structure that typically includes:

- **Torso**: Central body with head, arms, and legs attached
- **Head**: Usually contains cameras and sensors
- **Arms**: Shoulders, elbows, wrists, and hands
- **Legs**: Hips, knees, ankles, and feet

### Example Humanoid Robot URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- Similar joints and links for right arm, legs, etc. -->
</robot>
```

## Model Visualization Concepts

Robot models can be visualized in several ways:

### RViz
RViz is ROS's 3D visualization tool that can display robot models using the RobotModel plugin. It reads the robot description from the `/robot_description` parameter.

### Gazebo
Gazebo is a physics simulation environment that can load URDF files and provide realistic physics simulation.

### Web-based Visualization
Tools like MeshCat can visualize robot models in web browsers.

## Creating Your First Robot Description

Let's create a simple differential drive robot as an example:

```xml
<?xml version="1.0"?>
<robot name="diff_drive_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.4"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Right wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Castor wheel -->
  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.2 0 0"/>
  </joint>

  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Best Practices for Robot Description

1. **Start Simple**: Begin with a basic model and add complexity gradually
2. **Use Realistic Dimensions**: Base link sizes and joint positions on actual robot measurements
3. **Check Kinematic Chain**: Ensure all links are connected in a proper tree structure
4. **Validate URDF**: Use tools like `check_urdf` to validate your robot description
5. **Use Standard Names**: Follow ROS conventions for joint and link names

## Summary

In this chapter, we've explored robot description formats, particularly URDF, which is essential for representing robot models in ROS. We've learned how to define links, joints, and sensors, and how to create basic robot descriptions for humanoid robots. Understanding these concepts is crucial for robot simulation, visualization, and control.

## Next Steps

Previous: [Chapter 2: Robot Control Programming](../chapter2-control)

Congratulations! You have completed the ROS 2 Robotics Module. You now have a solid foundation in ROS 2 concepts, implementation, and robot modeling.

## Exercises

<Exercise title="Exercise 1: Basic Robot Model" difficulty="beginner">
Create a URDF file for a simple robot with:
1. A base link (box shape)
2. Two wheels (cylinder shapes) connected with continuous joints
3. Apply appropriate colors to different parts
4. Validate your URDF file using online tools or ROS tools

Make sure your robot has a proper kinematic tree structure.
</Exercise>

<Exercise title="Exercise 2: Humanoid Arm" difficulty="intermediate">
Design a simple humanoid arm with:
1. Shoulder, elbow, and wrist joints
2. Proper link dimensions representing an actual arm
3. At least 3 revolute joints to provide full range of motion
4. Visual elements showing the arm structure

Focus on getting the kinematic chain correct.
</Exercise>

<Exercise title="Exercise 3: Complete Humanoid Model" difficulty="advanced">
Create a simplified humanoid robot model with:
1. Torso, head, and limbs
2. Proper joint limits for realistic movement
3. Basic collision geometry
4. A visualization-ready model

This should be a complete, albeit simplified, representation of a humanoid robot.
</Exercise>

## Further Reading

- [URDF/XML Format Documentation](http://wiki.ros.org/urdf/XML)
- [Working with URDF in ROS 2](https://docs.ros.org/en/rolling/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher.html)
- [xacro Preprocessor](http://wiki.ros.org/xacro)
- [Robot State Publisher](https://docs.ros.org/en/rolling/p/robot_state_publisher/)
- [RViz Visualization](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/Custom-RViz-Plugins.html)