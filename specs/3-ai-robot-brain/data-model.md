# Data Model: AI-Robot Brain Module

## Chapter Document Entity

### Core Attributes
- **id** (string, required): Unique identifier for the document, following format "module-3/[chapter-name]"
- **title** (string, required): Display title of the chapter
- **description** (string, required): Brief description of the chapter content
- **slug** (string, required): URL-friendly version of the title
- **sidebar_label** (string, required): Label to appear in the sidebar navigation

### Content Structure
- **learning_objectives** (array of strings, required): List of specific learning outcomes for the chapter
- **prerequisites** (array of strings, optional): Knowledge or skills students should have before reading
- **content_sections** (array of objects, required): Structured content sections
  - **section_title** (string, required): Title of the content section
  - **section_content** (string, required): Markdown content for the section
  - **section_type** (string, required): Type of content (e.g., "concept", "example", "exercise")
- **code_examples** (array of objects, optional): Programming examples
  - **language** (string, required): Programming language identifier (e.g., "python", "c++")
  - **code** (string, required): The actual code example
  - **explanation** (string, required): Explanation of what the code does
- **exercises** (array of objects, optional): Practical exercises for students
  - **exercise_title** (string, required): Title of the exercise
  - **exercise_description** (string, required): Detailed description of the exercise
  - **difficulty_level** (string, required): "beginner", "intermediate", or "advanced"
  - **estimated_time** (integer, required): Estimated time to complete in minutes
- **summary** (string, required): Chapter summary recapping key concepts
- **further_reading** (array of objects, optional): Additional resources
  - **title** (string, required): Title of the resource
  - **url** (string, required): URL to the resource
  - **description** (string, required): Brief description of the resource

### Metadata
- **authors** (array of strings, optional): Authors of the content
- **last_updated** (string, required): ISO date string of last update
- **version** (string, required): Version of the content
- **tags** (array of strings, optional): Tags for categorization (e.g., "isaac", "ros2", "navigation", "gpu-acceleration")

### Validation Rules
- All required fields must be present
- The id must be unique across all documents
- The slug must contain only lowercase letters, numbers, and hyphens
- Learning objectives must be specific and measurable
- Content sections must have meaningful titles
- Code examples must be syntactically valid
- Difficulty levels for exercises must be one of the allowed values

## Navigation Entity

### Sidebar Category
- **type** (string, required): Must be "category" for categories
- **label** (string, required): Display name for the category in navigation
- **items** (array of strings/objects, required): List of document IDs or nested categories
- **collapsed** (boolean, optional): Whether the category is collapsed by default (default: false)

### Sidebar Document Link
- **type** (string, required): Must be "doc" for document links
- **id** (string, required): The document ID that corresponds to a file path

### Validation Rules
- All category items must correspond to existing documents or valid nested categories
- Document IDs in navigation must match actual file paths
- Labels must be human-readable and concise
- Navigation structure must be logically organized

## Chapter-Specific Entities

### Isaac Sim Chapter Data
- **simulation_features_covered** (array of strings): Isaac Sim features discussed (e.g., "gpu-accelerated-physics", "sensor-simulation", "perception")
- **robot_models_included** (array of strings): Types of robot models discussed (e.g., "humanoid", "wheeled", "manipulator")
- **simulation_parameters** (object): Key parameters for Isaac Sim
  - **rendering_quality** (string, optional): Quality level of rendering ("low", "medium", "high", "ultra")
  - **physics_accuracy** (number, optional): Physics simulation accuracy factor
  - **gpu_utilization** (number, optional): Expected GPU utilization percentage

### Isaac ROS Chapter Data
- **ros_packages_used** (array of strings): ROS 2 packages referenced (e.g., "isaac_ros_bridges", "isaac_ros_perception", "ros2_control")
- **message_types_covered** (array of strings): ROS message types discussed (e.g., "sensor_msgs", "geometry_msgs", "nav_msgs")
- **integration_patterns** (array of strings): Integration patterns (e.g., "message-translation", "service-calls", "action-libraries")

### Nav2 Navigation Chapter Data
- **navigation_algorithms_covered** (array of strings): Navigation algorithms discussed (e.g., "dijkstra", "astar", "teb", "dwb")
- **planner_types** (array of strings): Types of planners (e.g., "global-planner", "local-planner", "controller")
- **gpu_acceleration_techniques** (array of strings): GPU acceleration methods (e.g., "cuda", "tensorrt", "optix")

## Relationships
- Each Chapter Document belongs to exactly one Module (Module 3)
- Each Chapter Document appears in the Navigation structure once
- Chapter Documents may reference external resources in further_reading
- Navigation structure reflects the hierarchical organization of the course content