# Quickstart Guide: AI-Robot Brain Module Development

## Overview
This guide provides a quick setup process for contributing to the AI-Robot Brain Module (Module 3) for NVIDIA Isaac™ education. The module consists of three chapters covering Isaac Sim, Isaac ROS, and Nav2 Navigation.

## Prerequisites
- Node.js (v16 or higher)
- Git
- Basic understanding of Docusaurus documentation framework
- Basic ROS 2 knowledge
- Access to NVIDIA Isaac™ and Nav2 official documentation for reference

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

## Creating Module 3 Content

### 1. Create Chapter Files
Create the three required chapter files in the `docs/module-3/` directory:

```bash
# Create the module directory
mkdir -p docs/module-3

# Create the three chapter files
touch docs/module-3/isaac-sim.md
touch docs/module-3/isaac-ros.md
touch docs/module-3/nav2-navigation.md
```

### 2. Add Content to Each Chapter
Each chapter should follow this basic structure:

```markdown
---
id: isaac-sim
title: Isaac Sim - GPU-Accelerated Robotics Simulation
sidebar_label: Isaac Sim
description: Learn about NVIDIA Isaac™ simulation for robotics
---

## Learning Objectives
- Understand Isaac Sim architecture and capabilities
- Learn about GPU-accelerated physics simulation
- Explore sensor simulation techniques

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
    "Module 3 - AI-Robot Brain": [
      "module-3/isaac-sim",
      "module-3/isaac-ros",
      "module-3/nav2-navigation"
    ],
    // ... other modules
  },
};
```

## Content Creation Guidelines

### For Isaac Sim Chapter
- Focus on GPU-accelerated physics simulation
- Cover sensor simulation and perception capabilities
- Include Isaac Sim architecture and tools
- Provide practical exercises with simulation scenarios

### For Isaac ROS Chapter
- Cover Isaac ROS bridges and message translation
- Explain integration patterns with ROS 2
- Include perception module usage
- Provide practical examples with ROS 2 nodes

### For Nav2 Navigation Chapter
- Cover Nav2 architecture and planners
- Explain GPU-accelerated navigation algorithms
- Include simulation-to-reality transfer techniques
- Provide practical navigation exercises

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

3. **Educational Focus**: Content should be appropriate for AI/CS students with basic ROS 2 knowledge.

4. **Technical Accuracy**: All information should be based on official NVIDIA Isaac™ and Nav2 documentation.

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