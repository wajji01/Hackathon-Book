# Quickstart Guide: Digital Twin Module Development

## Overview
This guide provides a quick setup process for contributing to the Digital Twin Module (Module 2) for robotics simulation education. The module consists of three chapters covering Gazebo Physics, Unity Environments, and Simulated Sensors.

## Prerequisites
- Node.js (v16 or higher)
- Git
- Basic understanding of Docusaurus documentation framework
- Access to Gazebo and Unity official documentation for reference

## Environment Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
cd website  # or wherever the Docusaurus config is located
npm install
```

### 3. Start Local Development Server
```bash
npm run start
```
This will start the Docusaurus development server at http://localhost:3000

## Creating Module 2 Content

### 1. Create Chapter Files
Create the three required chapter files in the `docs/module-2/` directory:

```bash
# Create the module directory
mkdir -p docs/module-2

# Create the three chapter files
touch docs/module-2/gazebo-physics.md
touch docs/module-2/unity-environments.md
touch docs/module-2/simulated-sensors.md
```

### 2. Add Content to Each Chapter
Each chapter should follow this basic structure:

```markdown
---
id: gazebo-physics
title: Gazebo Physics - Physics-Based Simulation for Humanoid Robots
sidebar_label: Gazebo Physics
description: Learn about physics-based simulation in Gazebo for humanoid robots
---

## Learning Objectives
- Understand physics engines in Gazebo
- Learn about collision detection and response
- Explore rigid body dynamics for humanoid robots

## Content Sections
[Your content here]

## Exercises
[Practical exercises for students]

## Summary
[Chapter summary]

## Further Reading
[Links to additional resources]
```

### 3. Update Sidebar Configuration
Add the new module to the `sidebars.js` file:

```javascript
module.exports = {
  docs: {
    // ... other modules
    "Module 2 - Digital Twin": [
      "module-2/gazebo-physics",
      "module-2/unity-environments",
      "module-2/simulated-sensors"
    ],
    // ... other modules
  },
};
```

## Content Creation Guidelines

### For Gazebo Physics Chapter
- Focus on physics engines (ODE, Bullet, Simbody)
- Cover collision detection and response
- Explain rigid body dynamics and constraints
- Include joint simulation concepts
- Provide practical exercises with humanoid robots

### For Unity Environments Chapter
- Cover Unity's physics system
- Explain scene setup for robotics
- Discuss Unity Robotics packages
- Include robot model integration
- Provide practical examples

### For Simulated Sensors Chapter
- Cover various sensor types (cameras, LIDAR, IMU)
- Explain noise modeling and accuracy
- Discuss sensor integration with perception
- Include practical perception exercises

## Building and Testing

### Build the Documentation
```bash
npm run build
```

### Serve the Built Documentation
```bash
npm run serve
```

### Check for Broken Links
```bash
npm run docusaurus write-heading-ids  # to regenerate heading IDs if needed
```

## Best Practices

1. **Content Structure**: Each chapter should have clear learning objectives, content sections, practical exercises, and a summary.

2. **Code Examples**: All code examples should be verified as runnable and include explanations.

3. **Educational Focus**: Content should be appropriate for AI/CS students with basic programming knowledge.

4. **Technical Accuracy**: All information should be based on official Gazebo and Unity documentation.

5. **Navigation**: Ensure all chapters are properly registered in the sidebar for easy navigation.

## Troubleshooting

### Chapter Not Appearing in Navigation
- Verify the document ID matches what's in `sidebars.js`
- Check that the file is in the correct directory
- Ensure the file has the proper frontmatter

### Broken Links
- Use relative links within the documentation
- Verify all referenced files exist
- Check that document IDs are consistent

### Build Errors
- Ensure all required frontmatter fields are present
- Check for proper markdown syntax
- Verify file paths are correct