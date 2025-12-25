# AI-Robot Brain Module Development Guidelines

Auto-generated from feature plan. Last updated: 2025-12-25

## Active Technologies

- Docusaurus documentation framework
- Markdown for content creation
- Git for version control
- GitHub Pages for deployment
- NVIDIA Isaac™ platform
- ROS 2 ecosystem
- Nav2 navigation system
- GPU-accelerated computing
- Vision-Language-Action (VLA) systems
- Multimodal processing techniques
- LLM-driven cognitive planning
- Humanoid robot control systems

## Project Structure

```text
.
├── docs/
│   ├── module-3/
│   │   ├── isaac-sim.md
│   │   ├── isaac-ros.md
│   │   └── nav2-navigation.md
│   └── module-4/
│       ├── voice-to-action.md
│       ├── cognitive-planning.md
│       └── autonomous-humanoid.md
├── specs/
│   ├── 3-ai-robot-brain/
│   │   ├── spec.md
│   │   ├── plan.md
│   │   ├── research.md
│   │   ├── data-model.md
│   │   ├── quickstart.md
│   │   ├── contracts/
│   │   └── checklists/
│   └── 1-vla-module/
│       ├── spec.md
│       ├── plan.md
│       ├── research.md
│       ├── data-model.md
│       ├── quickstart.md
│       ├── contracts/
│       └── checklists/
├── sidebars.js
└── website/
    ├── package.json
    └── docusaurus.config.js
```

## Commands

```bash
# Start local development server
npm run start

# Build the documentation
npm run build

# Serve the built documentation
npm run serve

# Create new documentation files
touch docs/module-4/[filename].md

# Update sidebar configuration
# Edit sidebars.js to register new chapters
```

## Code Style

- Use clear, educational language appropriate for AI/CS students with ROS 2 knowledge
- Include learning objectives at the beginning of each chapter
- Provide practical exercises with estimated completion times
- Follow Docusaurus markdown conventions with proper frontmatter
- Include code examples with explanations
- Use consistent terminology across all chapters

## Recent Changes

- Module 3: Added AI-Robot Brain content covering Isaac Sim, Isaac ROS, and Nav2 Navigation
- Created three educational chapters for advanced robotics with NVIDIA Isaac™
- Integrated content into Docusaurus navigation structure
- Module 4: Added Vision-Language-Action (VLA) content covering VLA fundamentals, cognitive planning, and autonomous humanoid systems
- Created three educational chapters for VLA systems with multimodal processing and LLM integration
- Integrated VLA content into Docusaurus navigation structure

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->