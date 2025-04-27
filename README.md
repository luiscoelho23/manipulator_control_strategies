# Manipulator Control Strategies

A ROS2 package implementing various control strategies for the FRANKA EMIKA Panda robot, focused on Dynamic Movement Primitives (DMPs) execution.

## Overview

This package provides modules for:
- Loading and executing DMPs on the robot
- Collision detection during trajectory execution
- Integration with reinforcement learning models to adapt to obstacles

## Architecture

The package integrates with other components in the manipulator workspace:
- Uses `manipulator` package for robot control and kinematics
- Uses `manipulator_skill_acquisition` for reinforcement learning models
- Works with `mplibrary` for motion primitives

## Dependencies

### Core Dependencies
- ROS2 Humble
- Python 3.8+
- rclpy
- PyKDL
- manipulator package
- manipulator_skill_acquisition package
- mplibrary (C++ library with Python bindings)

### System Requirements
- Ubuntu 22.04 LTS or higher
- ROS2 Humble

## Installation

1. Clone the repository into your workspace:
   ```bash
   cd ~/ws_manipulator/src
   git clone https://github.com/username/manipulator_control_strategies.git
   ```

2. Build the workspace:
   ```bash
   cd ~/ws_manipulator
   colcon build --packages-select manipulator_control_strategies manipulator manipulator_skill_acquisition
   source install/setup.bash
   ```

## Usage

### Loading DMPs

The package provides a script for loading and executing DMPs:

```bash
ros2 run manipulator_control_strategies load_dmps.py
```

This will:
1. Load a DMP from a saved model
2. Execute the trajectory on the robot
3. Perform collision detection during execution
4. Apply reinforcement learning adaptations when obstacles are detected

### Integration with Other Packages

This package is designed to work with:

- **manipulator**: Provides the robot control interface and collision detection
- **manipulator_skill_acquisition**: Provides trained RL models for trajectory adaptation
- **manipulator_gui**: Provides a GUI for visualizing and configuring control strategies

## Implementation Details

The `load_dmps.py` module:
- Subscribes to robot joint states
- Retrieves the robot description from ROS parameters
- Performs forward kinematics to compute end-effector positions
- Interfaces with collision detection through the C++ backend
- Uses reinforcement learning models to adapt trajectories when obstacles are detected

## License

This project is licensed under the Apache License 2.0. 