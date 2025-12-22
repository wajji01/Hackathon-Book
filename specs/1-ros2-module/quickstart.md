# Quickstart: ROS 2 Robotics Module

## Prerequisites
- Node.js v18 or higher
- npm or yarn package manager
- Git for version control

## Installation

1. **Install Docusaurus**:
   ```bash
   npx create-docusaurus@latest website ros2-robotics-module --typescript throwaway
   ```

2. **Navigate to project directory**:
   ```bash
   cd ros2-robotics-module
   ```

3. **Start development server**:
   ```bash
   npm start
   ```

4. **Create module structure**:
   ```bash
   mkdir -p docs/modules/ros2-module
   ```

## Creating the Three Chapters

1. **Chapter 1: ROS 2 Fundamentals**
   ```bash
   # Create the fundamentals chapter
   cat > docs/modules/ros2-module/chapter1-fundamentals.md << 'EOF'
---
sidebar_position: 1
title: "ROS 2 Fundamentals"
---

# ROS 2 Fundamentals

This chapter covers the core concepts of ROS 2 including its purpose, architecture, and communication patterns.

## Learning Objectives
- Understand the purpose and architecture of ROS 2
- Identify the four communication patterns: nodes, topics, services, and actions
- Explain when to use each communication pattern

## Content to be added...
EOF
   ```

2. **Chapter 2: Robot Control Programming**
   ```bash
   # Create the control chapter
   cat > docs/modules/ros2-module/chapter2-control.md << 'EOF'
---
sidebar_position: 2
title: "Robot Control Programming"
---

# Robot Control Programming

This chapter covers how to control robotic systems using programming interfaces, including creating publishers, subscribers, and services.

## Learning Objectives
- Create publishers and subscribers to robot communication topics
- Understand how to bridge AI logic to robot controllers
- Implement basic communication patterns in code

## Content to be added...
EOF
   ```

3. **Chapter 3: Robot Description and Modeling**
   ```bash
   # Create the modeling chapter
   cat > docs/modules/ros2-module/chapter3-urdf.md << 'EOF'
---
sidebar_position: 3
title: "Robot Description and Modeling"
---

# Robot Description and Modeling

This chapter covers how to describe humanoid robots using robot description formats, including defining links, joints, and sensors.

## Learning Objectives
- Create robot descriptions with links and joints
- Understand visualization and simulation concepts
- Define sensors in robot models

## Content to be added...
EOF
   ```

## Sidebar Configuration

Update `sidebars.js` to include the new module:

```javascript
// sidebars.js
module.exports = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 Robotics Module',
      items: [
        'modules/ros2-module/chapter1-fundamentals',
        'modules/ros2-module/chapter2-control',
        'modules/ros2-module/chapter3-urdf',
      ],
    },
  ],
};
```

## Running the Documentation Site

1. **Start development server**:
   ```bash
   npm start
   ```

2. **Build for production**:
   ```bash
   npm run build
   ```

3. **Serve production build locally**:
   ```bash
   npm run serve
   ```

## Deployment

The site can be deployed to GitHub Pages using Docusaurus' built-in deployment command:

```bash
GIT_USER=<Your GitHub username> USE_SSH=true npm run deploy
```