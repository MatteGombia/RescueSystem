# RescueSystem

An autonomous rescue system that uses the Cozmo robot to explore an environment, detect obstacles (walls), locate light cubes, and retrieve them to a safe location.

## Overview

This project implements a sophisticated navigation and exploration system for the Cozmo robot that:
- **Autonomously explores** unknown environments using position-based navigation
- **Detects obstacles** using image analysis and custom wall markers
- **Avoids collisions** with dynamic path planning and wall avoidance
- **Locates and rescues** light cubes by navigating to them and returning them to the origin
- **Visualizes progress** with real-time map plotting showing walls, path, and cube locations

## Project Structure

```
├── rescue.py          # Main rescue system implementation
├── frame2d.py         # 2D frame/pose representation utilities
└── README.md          # This file
```

## Key Features

### Navigation System
- **Position-based exploration**: Tracks visited positions and systematically explores unvisited areas
- **Wall avoidance**: Detects walls through image analysis and calculates avoidance paths
- **Collision detection**: Uses image difference thresholding to detect obstacles
- **Precise rotation**: Maintains heading accuracy within configurable angle thresholds

### Obstacle Detection
- Image-based wall detection using camera feed differences
- Custom wall object recognition through marker detection
- Wall angle tracking for realistic navigation around obstacles

### Cube Rescue
- Scans for light cubes at configurable step intervals (360° coverage)
- Navigates to detected cubes while avoiding obstacles
- Retrieves cubes using the robot's lift mechanism
- Returns cubes to the origin point (0, 0)

### Visualization
- Real-time matplotlib plotting of:
  - Robot position and heading direction
  - Explored path (green circles)
  - Detected walls (red squares with line segments)
  - Located cubes (yellow squares)
  - Elapsed time and discovery counters

## Configuration Parameters

Key constants that can be tuned:

```python
WHEEL_SPEED = 250                 # Motor speed (0-255)
DISTANCE_PER_MOVE = 100          # Distance per movement step (mm)
MOVE_DISTANCE = 400              # Distance between exploration positions (mm)
WALL_RADIUS = 100                # Collision detection radius (mm)
WALL_THRESHOLD = 50              # Wall avoidance buffer (mm)
IMAGE_DIFF_THRESHOLD = 8         # Wall detection sensitivity
CALIBRATED_CONSTANT = 2.25       # Movement duration calibration factor
ANGLE_THRESHOLD = 0.05           # Rotation accuracy threshold (radians)
MIN_ROTATION_SEC = 0.30          # Minimum rotation duration (seconds)
```

## Dependencies

- `cozmo`: Cozmo SDK for robot control
- `numpy`: Numerical computing
- `matplotlib`: Visualization and plotting
- Custom `frame2d` module for pose transformations

## Usage

Run the rescue system with:

```bash
python rescue.py
```

The program will:
1. Initialize the Cozmo robot and custom wall definitions
2. Begin exploring the environment autonomously
3. Scan for light cubes at each position
4. Navigate to and rescue any discovered cubes
5. Continue exploring until interrupted with Ctrl+C

Press Ctrl+C to stop the program and view final statistics.

## Algorithm Details

### Wall Avoidance
When a wall is detected, the system:
1. Identifies the wall's position and orientation
2. Calculates a point at safe distance from the wall (perpendicular to wall orientation)
3. Recursively navigates to the avoidance point before continuing to the target

### Path Planning
- Checks if direct paths to target positions are blocked by known walls
- If blocked, selects alternative positions with maximum clearance
- Uses a clearance buffer to ensure safe navigation around obstacles

### Cube Detection
- Performs 360° stepper scan from current position
- Logs cube positions in the global `cubes` dictionary
- Tracks which cubes have been rescued

## Statistics

Upon completion, the system prints:
- Total elapsed time
- Total distance traveled
- Number of walls detected
- Number of cubes found and rescued

## Notes

- The system operates with a global map of explored positions to avoid redundant exploration
- Wall angles are tracked separately to enable proper avoidance calculations
- The robot maintains continuous image stream for real-time obstacle detection
- All coordinates are in millimeters relative to the robot's starting position
