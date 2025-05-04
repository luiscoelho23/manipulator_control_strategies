# Manipulator Control Strategies

A ROS2 package implementing various control strategies for the FRANKA EMIKA Panda robot, focused on Dynamic Movement Primitives (DMPs) execution.

## Overview

This package provides modules for:
- Loading and executing DMPs on the robot
- Sending direct position commands to specific controllers
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
- [mplibrary](https://github.com/BiRDLab-UMinho/mplibrary)

### System Requirements
- Ubuntu 22.04 LTS or higher
- ROS2 Humble

## Installation

1. Clone the repository into your workspace:
   ```bash
   cd ~/ws_manipulator/src
   git clone https://github.com/luiscoelho23/manipulator_control_strategies.git
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
ros2 run manipulator_control_strategies load_dmps.py [MPX_PATH] [RL_ACTOR_PATH] [--debug]
```

This will:
1. Load a DMP from a saved model
2. Execute the trajectory on the robot
3. Perform collision detection during execution
4. Apply reinforcement learning adaptations when obstacles are detected

Arguments:
- `MPX_PATH`: Path to the motion primitive library file (optional)
- `RL_ACTOR_PATH`: Path to the RL actor model (optional)
- `--debug`: Enable debug mode, which includes trajectory controller testing and verbose logging

### Sending Position Commands

The package also provides a script for sending direct position commands to the robot:

```bash
ros2 run manipulator_control_strategies send_to_pos.py <j1> <j2> <j3> <j4> <j5> <j6> <j7> [--debug] [--controller=TYPE] [--mode=MODE]
```

This allows you to:
1. Send joint positions directly to any controller
2. Choose between different controller types
3. Test controller functionality
4. Specify position or effort control mode

Arguments:
- `j1-j7`: Joint angles/efforts in radians/Nm (required)
- `--debug`: Enable debug mode for verbose logging
- `--controller=TYPE`: Specify controller type to use
  - Options: `auto`, `forward_position_controller`, `forward_effort_controller`, `joint_trajectory_controller_position`, `joint_trajectory_controller_effort`
- `--mode=MODE`: Specify control mode (`position` or `effort`)

Examples:
```bash
# Send to home position using auto-detected controller
ros2 run manipulator_control_strategies send_to_pos.py 0 0 0 0 0 0 0

# Use specific controller with debug output
ros2 run manipulator_control_strategies send_to_pos.py 0 -0.3 0 -1.8 0 1.5 0 --controller=forward_position_controller --debug

# Use effort control mode
ros2 run manipulator_control_strategies send_to_pos.py 0 10 0 5 0 2 0 --controller=forward_effort_controller --mode=effort
```

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

The `send_to_pos.py` module:
- Provides a simple interface to send positions to any robot controller
- Auto-detects available controllers or uses specified controller
- Supports both position and effort control modes
- Can be used for testing or as part of an application

## License

This project is licensed under the Apache License 2.0. 