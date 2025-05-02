#!/usr/bin/python3

import sys
import os
import time
import rclpy
from rclpy.node import Node
import rclpy.logging
from manipulator import controller

def main():
    # Initialize ROS
    rclpy.init()
    
    # Create logger
    logger = rclpy.logging.get_logger('send_to_pos')
    
    # Parse command line arguments
    if len(sys.argv) < 8:
        logger.error("Usage: send_to_pos.py <j1> <j2> <j3> <j4> <j5> <j6> <j7> [--debug] [--controller=TYPE]")
        logger.error("       All joint values should be in radians")
        logger.error("       Controller types: auto, forward_position_controller, forward_effort_controller,")
        logger.error("                         joint_trajectory_controller_position, joint_trajectory_controller_effort")
        sys.exit(1)
    
    # Extract joint angles from arguments
    try:
        joint_angles = [float(sys.argv[i]) for i in range(1, 8)]
        logger.info(f"Sending robot to position: {[f'{a:.3f}' for a in joint_angles]}")
    except ValueError:
        logger.error("Error: All joint values must be numbers")
        sys.exit(1)
    
    # Check for debug flag and controller type
    debug_mode = False
    controller_type = 'auto'
    control_mode = 'position'  # Default control mode
    
    # Process additional arguments
    for arg in sys.argv[8:]:
        if arg == "--debug":
            debug_mode = True
            logger.info("Debug mode enabled")
        elif arg.startswith("--controller="):
            controller_type = arg.split("=")[1]
            logger.info(f"Using specified controller: {controller_type}")
        elif arg.startswith("--mode="):
            control_mode = arg.split("=")[1]
            logger.info(f"Using specified control mode: {control_mode}")
    
    try:
        # Create controller client with specified parameters
        client = controller.ControllerClient(
            ts=0.1,
            controller_type=controller_type,
            control_mode=control_mode
        )
        client.set_debug_mode(debug_mode)
        
        if debug_mode:
            logger.info(f"Using controller type: {client.controller_type}")
            logger.info(f"Using control mode: {client.control_mode}")
            
            # Check if specific controller is available
            if client.controller_type != controller_type and controller_type != 'auto':
                logger.warning(f"Requested controller {controller_type} not available, using {client.controller_type} instead")
        
        # Wait a moment for controller to initialize
        time.sleep(1.0)
        
        # Send goal
        logger.info("Sending goal to controller...")
        
        # For forward controllers, we can send goals directly
        # For trajectory controllers, we need more parameters
        if debug_mode:
            if 'forward' in client.controller_type:
                logger.info(f"Sending direct command to {client.controller_type}")
            else:
                logger.info(f"Sending trajectory to {client.controller_type}")
        
        client.send_goal(*joint_angles, duration=3.0)
        logger.info("Goal sent!")
        
        # Wait for motion to complete - longer for trajectory controllers
        wait_time = 5.0 if 'trajectory' in client.controller_type else 3.0
        logger.info(f"Waiting {wait_time} seconds for motion to complete...")
        time.sleep(wait_time)
        
        # Clean up
        client.destroy_node()
        logger.info("Done")
        
    except Exception as e:
        logger.error(f"Error: {str(e)}")
        if debug_mode:
            import traceback
            logger.error(traceback.format_exc())
    finally:
        # Ensure proper shutdown
        rclpy.shutdown()

if __name__ == "__main__":
    main()