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
- mplibrary (C++ library with Python bindings)

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

## Controller Configuration

The package supports several controller types for different control approaches:

### Controller Types

1. **Forward Position Controller**
   - Direct joint position control
   - Low-level interface with minimal interpolation
   - Fastest response time
   - Example: `--controller=forward_position_controller`

2. **Forward Effort Controller**
   - Direct joint effort/torque control
   - Used for impedance control or force-based interactions
   - Requires `--mode=effort` parameter
   - Example: `--controller=forward_effort_controller --mode=effort`

3. **Joint Trajectory Controller**
   - Smooth trajectory execution with proper interpolation
   - Respects velocity and acceleration limits
   - Available in position and effort variants
   - Examples:
     - `--controller=joint_trajectory_controller_position`
     - `--controller=joint_trajectory_controller_effort --mode=effort`

### Auto Detection

By default, both scripts will auto-detect available controllers. The detection follows this priority:
1. Trajectory controllers (preferred for smoother motion)
2. Forward controllers (as fallback for direct control)

To see which controllers are available on your system:
```bash
ros2 control list_controllers
```

## Debug Mode

Both scripts support a debug mode that provides enhanced information and checks. When enabled with the `--debug` flag:

### In load_dmps.py:
- Runs a test of the trajectory controller before starting the DMP
- Provides detailed logging of trajectory points
- Shows obstacle avoidance activation details
- Reports timing statistics for execution
- Includes controller type and mode information

### In send_to_pos.py:
- Shows controller auto-detection details
- Reports controller type and mode being used
- Displays trajectory details when using trajectory controllers
- Includes error tracebacks when exceptions occur

## Integration with Other Packages

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

## Troubleshooting

### Common Issues

1. **Script not found when using ros2 run**
   - Make sure you've built the package with `colcon build`
   - Source your workspace: `source ~/ws_manipulator/install/setup.bash`
   - Check file permissions: `chmod +x ~/ws_manipulator/src/manipulator_control_strategies/manipulator_control_strategies/script_name.py`
   - Rebuild if needed: `colcon build --packages-select manipulator_control_strategies`

2. **Controller not found or inactive**
   - Check available controllers: `ros2 control list_controllers`
   - Ensure the robot is properly launched: `ros2 launch manipulator launch.py`
   - Try activating the controller manually: `ros2 control switch_controllers --activate joint_trajectory_controller_position`

3. **CRLF/LF Line Ending Issues**
   - If you see errors like `/usr/bin/env: 'python3\r': No such file or directory`, convert to Unix line endings:
     ```bash
     sudo apt install dos2unix
     dos2unix ~/ws_manipulator/install/manipulator_control_strategies/lib/manipulator_control_strategies/*.py
     ```

4. **Import Errors**
   - If you see `ModuleNotFoundError`, check that all dependencies are installed and the Python environment is correct
   - Try rebuilding the workspace with `--symlink-install`: `colcon build --symlink-install --packages-select manipulator_control_strategies`

### Debugging Tips

1. Always try running with `--debug` flag first to get more detailed information
2. Check the terminal output for errors or warnings
3. Verify that controllers are active before sending commands
4. For collision detection issues, ensure the robot description parameter is available

## License

This project is licensed under the Apache License 2.0. 