# Data Model: Digital Twin Module

## Chapter Document Entity

### Core Attributes
- **id** (string, required): Unique identifier for the document, following format "module-2/[chapter-name]"
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
  - **language** (string, required): Programming language identifier (e.g., "python", "csharp")
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
- **tags** (array of strings, optional): Tags for categorization (e.g., "gazebo", "unity", "physics", "sensors")

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

### Gazebo Physics Chapter Data
- **physics_engines_supported** (array of strings): List of supported physics engines (e.g., "ODE", "Bullet", "Simbody")
- **joint_types_covered** (array of strings): Types of joints discussed (e.g., "revolute", "prismatic", "fixed", "ball", "universal")
- **simulation_parameters** (object): Key parameters for physics simulation
  - **gravity** (number, optional): Gravitational acceleration value
  - **real_time_factor** (number, optional): Simulation speed relative to real time
  - **max_step_size** (number, optional): Maximum physics step size

### Unity Environments Chapter Data
- **unity_packages_used** (array of strings): Unity packages referenced (e.g., "Unity Physics", "Unity Robotics", "ROS#")
- **environment_types** (array of strings): Types of environments covered (e.g., "indoor", "outdoor", "industrial")
- **robot_models_included** (array of strings): Types of robot models discussed (e.g., "humanoid", "wheeled", "manipulator")

### Simulated Sensors Chapter Data
- **sensor_types_covered** (array of strings): Types of sensors discussed (e.g., "camera", "lidar", "imu", "gps", "force_torque")
- **sensor_accuracy_factors** (object): Factors affecting sensor accuracy
  - **noise_model** (string, optional): Type of noise model used
  - **resolution** (number, optional): Sensor resolution parameter
  - **range** (number, optional): Effective range of the sensor
- **perception_algorithms** (array of strings): Algorithms that use sensor data (e.g., "SLAM", "object_detection", "path_planning")

## Relationships
- Each Chapter Document belongs to exactly one Module (Module 2)
- Each Chapter Document appears in the Navigation structure once
- Chapter Documents may reference external resources in further_reading
- Navigation structure reflects the hierarchical organization of the course content