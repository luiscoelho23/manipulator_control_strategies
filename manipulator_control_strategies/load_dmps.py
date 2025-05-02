#!/usr/bin/python3

import sys
import os
import numpy as np
import math
from std_msgs.msg import String
from manipulator import controller
from manipulator_skill_acquisition.rl import load_rl
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
import rclpy.logging
from sensor_msgs.msg import JointState

from threading import Thread
from manipulator import kdl_parser
import PyKDL as kdl
from urdf_parser_py.urdf import URDF

import subprocess
from ament_index_python.packages import get_package_share_directory

# Initialize rclpy right at the beginning before any nodes are created
rclpy.init()

# Create logger early to use it throughout the file
logger = rclpy.logging.get_logger('load_dmps')

current_file = os.path.abspath(__file__)
if '/install/' in current_file:
    ws_path = current_file[:current_file.find('/install/')]
elif '/src/' in current_file:
    ws_path = current_file[:current_file.find('/src/')]
else:
    logger.error("Could not determine workspace path. Script must be run from install or src directory.")
    sys.exit(1)

mplibrary_path = os.path.join(ws_path, 'build/mplibrary')

sys.path.append(mplibrary_path)
import pymplibrary as motion

# Pre-allocate joint angles array and frame for reuse
joint_angles = kdl.JntArray(7)
eeframe = kdl.Frame()

# Convert angles to radians once (optimization)
PI_OVER_180 = np.pi / 180
NEG_180_RAD = -180 * PI_OVER_180
ZERO_RAD = 0
POS_135_RAD = 135 * PI_OVER_180

def get_ee_position(ang1, ang2, ang3):
    # Set angles in radians directly to avoid repetitive conversions
    joint_angles[0] = NEG_180_RAD            # Joint 1 angle in radians
    joint_angles[1] = ang1 * PI_OVER_180     # Joint 2 angle in radians
    joint_angles[2] = ZERO_RAD               # Joint 3 angle in radians
    joint_angles[3] = NEG_180_RAD + ang2 * PI_OVER_180  # Joint 4 angle in radians
    joint_angles[4] = ZERO_RAD               # Joint 5 angle in radians
    joint_angles[5] = POS_135_RAD + ang3 * PI_OVER_180  # Joint 6 angle in radians
    joint_angles[6] = ZERO_RAD               # Joint 7 angle in radians
    
    fk_solver.JntToCart(joint_angles, eeframe)
    return eeframe.p.x(), eeframe.p.z()

class DMPSubscriber(Node):

    def __init__(self):
        super().__init__('dmp_subscriber')
        
        self.robot_description = None
        self.real_angle = [0, 0, 0]

        # Create client to get parameters from robot_state_publisher
        self.get_parameters_client = self.create_client(
            GetParameters,
            '/robot_state_publisher/get_parameters'
        )

        # Wait for service to be available
        while not self.get_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Parameter service not available, waiting...')

        # Get robot description parameter
        self.get_robot_description()

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        
        # Ensure the subscription is created
        self.get_logger().info('Created subscription to /joint_states')

    def get_robot_description(self):
        request = GetParameters.Request()
        request.names = ['robot_description']
        
        future = self.get_parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.values:
                self.robot_description = response.values[0].string_value
                self.get_logger().info('Got robot description parameter')
            else:
                self.get_logger().error('Failed to get robot_description parameter')
        else:
            self.get_logger().error('Service call failed')

    def listener_callback(self, msg):
        try:
            if self.robot_description != None:
                # Get all 7 joint positions
                joint_positions = list(msg.position)
                array_ob = [-0.15, 0, -0.1]
                
                # Convert all values to strings
                string1 = [str(x) for x in joint_positions]  # All 7 joint positions
                string2 = [str(x) for x in array_ob]  # 3 obstacle positions

                args = ["./run_cd"] + string1 + string2 + [self.robot_description]

                result = subprocess.run(
                    args,  
                    cwd= ws_path + "/build/manipulator",
                    text=True,                     # Capture output as text
                    capture_output=True,           # Capture stdout and stderr
                    check=True                     # Raise an exception on non-zero exit code
                )

                # Use logger instead of print
                logger.info(result.stdout)

        except subprocess.CalledProcessError as e:
            # Handle errors from the C++ program
            logger.error(f"Error executing C++ program: {e.stderr}")
            self.get_logger().error(f'Error in collision detection: {e.stderr}')
        except Exception as e:
            self.get_logger().error(f'Error in listener callback: {e}')

# Check controller status before sending goals
def ensure_controller_active():
    """Make sure the controller is active before sending goals"""
    logger.info("Ensuring controller is active...")
    
    try:
        # Check if controller_manager is available
        result = subprocess.run(
            ['ros2', 'control', 'list_controllers'],
            text=True,
            capture_output=True
        )
        
        logger.info(f"Controller status: {result.stdout}")
        
        # Check if trajectory controller is listed and active
        if 'joint_trajectory_controller_position' in result.stdout:
            if 'active' not in result.stdout:
                # Try to activate the controller
                logger.warning("Controller not active, attempting to activate...")
                act_result = subprocess.run(
                    ['ros2', 'control', 'switch_controllers', '--activate', 'joint_trajectory_controller_position'],
                    text=True,
                    capture_output=True
                )
                logger.info(f"Activation result: {act_result.stdout}")
                logger.info(f"Activation errors: {act_result.stderr}")
        else:
            logger.error("joint_trajectory_controller_position not found!")
            
        # Also check hardware interfaces
        hw_result = subprocess.run(
            ['ros2', 'control', 'list_hardware_interfaces'],
            text=True,
            capture_output=True
        )
        logger.info(f"Hardware interfaces: {hw_result.stdout}")
        
        # Check if hardware interfaces are claimed by the controller
        if 'claimed' not in hw_result.stdout:
            logger.warning("No hardware interfaces are claimed by controllers!")
            
    except Exception as e:
        logger.error(f"Error checking controller state: {e}")


def test_trajectory_controller():
    """Test the joint trajectory controller with simple movements"""
    logger.info("====== TESTING JOINT TRAJECTORY CONTROLLER ======")
    
    try:
        # Make sure controller is active
        result = subprocess.run(
            ['ros2', 'control', 'list_controllers'],
            text=True,
            capture_output=True
        )
        
        logger.info(f"Available controllers: {result.stdout}")
        
        if 'joint_trajectory_controller_position' not in result.stdout or 'active' not in result.stdout:
            # Try to activate it
            logger.warning("Trajectory controller not active, attempting to activate...")
            subprocess.run(
                ['ros2', 'control', 'switch_controllers', '--activate', 'joint_trajectory_controller_position'],
                text=True,
                capture_output=True
            )
        
        # Create a direct action client to test the trajectory controller
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from builtin_interfaces.msg import Duration
        
        # Create a temporary node for testing
        test_node = rclpy.create_node('trajectory_test_node')
        
        # Create an action client
        logger.info("Creating action client for trajectory controller...")
        action_client = ActionClient(
            test_node,
            FollowJointTrajectory,
            '/joint_trajectory_controller_position/follow_joint_trajectory'
        )
        
        # Wait for the action server
        if not action_client.wait_for_server(timeout_sec=2.0):
            logger.error("Could not connect to action server!")
            return False
            
        logger.info("Successfully connected to trajectory action server!")
        
        # Create a simple test trajectory - small movement up
        goal_msg = FollowJointTrajectory.Goal()
        
        # Set up the trajectory
        traj = JointTrajectory()
        traj.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 
                           'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        
        # Create two points - start (current position) and a target
        start_point = JointTrajectoryPoint()
        start_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Use zeros as a placeholder
        start_point.velocities = [0.0] * 7
        start_point.accelerations = [0.0] * 7
        start_point.time_from_start = Duration(sec=0, nanosec=10000000)  # 0.01 seconds
        
        # Create a simple target - move joint 2 slightly
        end_point = JointTrajectoryPoint()
        end_point.positions = [0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0]  # Small movement of joint 2
        end_point.velocities = [0.0] * 7
        end_point.accelerations = [0.0] * 7
        end_point.time_from_start = Duration(sec=2, nanosec=0)  # 2 seconds
        
        # Add points to trajectory
        traj.points = [start_point, end_point]
        
        # Set goal
        goal_msg.trajectory = traj
        
        # Set goal tolerance
        goal_msg.goal_time_tolerance = Duration(sec=1, nanosec=0)
        
        # Send goal
        logger.info("Sending test trajectory to trajectory controller...")
        future = action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(test_node, future, timeout_sec=1.0)
        goal_handle = future.result()
        
        if goal_handle.accepted:
            logger.info("Test trajectory was ACCEPTED!")
            
            # Wait a bit to see if movement happens
            logger.info("Waiting to see if robot moves...")
            time.sleep(3.0)
            
            # Try another simple movement - move back
            logger.info("Sending second test trajectory...")
            goal_msg2 = FollowJointTrajectory.Goal()
            traj2 = JointTrajectory()
            traj2.joint_names = traj.joint_names
            
            # Start from current theoretical position
            point1 = JointTrajectoryPoint()
            point1.positions = end_point.positions
            point1.velocities = [0.0] * 7
            point1.accelerations = [0.0] * 7
            point1.time_from_start = Duration(sec=0, nanosec=10000000)
            
            # Return to initial position
            point2 = JointTrajectoryPoint()
            point2.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            point2.velocities = [0.0] * 7
            point2.accelerations = [0.0] * 7
            point2.time_from_start = Duration(sec=2, nanosec=0)
            
            traj2.points = [point1, point2]
            goal_msg2.trajectory = traj2
            goal_msg2.goal_time_tolerance = Duration(sec=1, nanosec=0)
            
            future2 = action_client.send_goal_async(goal_msg2)
            rclpy.spin_until_future_complete(test_node, future2, timeout_sec=1.0)
            
            if future2.result().accepted:
                logger.info("Second test trajectory was ACCEPTED!")
            else:
                logger.error("Second test trajectory was REJECTED!")
                
            return future2.result().accepted
            
        else:
            logger.error("Test trajectory was REJECTED!")
            return False
            
    except Exception as e:
        logger.error(f"Error testing trajectory controller: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return False
    finally:
        # Clean up
        if 'test_node' in locals():
            test_node.destroy_node()

def dmp_load():
    
    main_traj = open("/home/luisc/main.csv", "w")
    main_ang_traj = open("/home/luisc/main_ang.csv", "w")
    
    # Pre-allocate with estimated size to avoid resizing
    estimated_points = int(1.0 / ts) + 100  # Add some margin
    main_trajectory_ang = np.zeros(estimated_points * 3)
    trajectory_index = 0

    x_ob, z_ob = -0.37, -0.25
    
    # Pre-compute some constants for the goal sending
    rad_neg180 = -180 * PI_OVER_180
    rad_0 = 0
    rad_135 = 135 * PI_OVER_180

    # Before starting, check interfaces
    ensure_controller_active()

    # Log debug information if debug mode is enabled
    if debug_mode:
        logger.info("=== DMP LOAD DEBUG INFO ===")
        logger.info(f"Using MPX file: {mpx_path}")
        logger.info(f"Using RL actor file: {rl_actor_path}")
        logger.info(f"Integration timestep: {ts}")
        logger.info(f"Obstacle position: ({x_ob}, {z_ob})")
        logger.info(f"Controller type: {client.controller_type}")
        logger.info(f"Control mode: {client.control_mode}")

    # Prepare the obstacle avoidance threshold check
    obstacle_check_threshold = 0.10
    obstacle_coordinates = np.array([x_ob, z_ob])
    
    # First pass to generate trajectory
    phase.value = 0.0
    policy.reset([8.683360515029568205e+01,1.148087923504335635e+02,2.452527404357190832e+01])

    if debug_mode:
        logger.info("Starting first pass trajectory generation")
        
    while phase.value < 0.999: 
        phase.update(ts)
        policy.update(ts, phase.value)

        if trajectory_index + 3 <= len(main_trajectory_ang):
            main_trajectory_ang[trajectory_index] = policy.value[0]
            main_trajectory_ang[trajectory_index + 1] = policy.value[1]
            main_trajectory_ang[trajectory_index + 2] = policy.value[2]
            trajectory_index += 3
        else:
            # Resize if needed (should be rare with good estimation)
            main_trajectory_ang = np.append(main_trajectory_ang, [policy.value[0], policy.value[1], policy.value[2]])
            trajectory_index += 3
            
        x, z = get_ee_position(policy.value[0], policy.value[1], policy.value[2])
        out = str(x) + ";" + str(z) + "\n"
        out1 = str(policy.value[0]) + ";" + str(policy.value[1]) + ";" + str(policy.value[2]) + "\n"
        main_traj.write(out)
        main_ang_traj.write(out1)
        
    main_traj.close()
    
    # Truncate to actual size used
    main_trajectory_ang = main_trajectory_ang[:trajectory_index]
    
    logger.info("DMP Loaded")
    if debug_mode:
        logger.info(f"Generated {trajectory_index // 3} trajectory points")

    subscriber = DMPSubscriber()
    # Create an executor for the subscriber
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(subscriber)

    # Start the subscriber thread
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    
    if debug_mode:
        logger.info("Joint state subscriber started")

    # Reset for second pass
    phase.value = 0
    policy.reset([8.683360515029568205e+01,1.148087923504335635e+02,2.452527404357190832e+01])
    traj_index = 0
    
    # Pre-allocate arrays for the main loop
    position_current = np.zeros(2)  # [x, z]
    position_target = np.zeros(2)   # [x_t, z_t]
    loop_start_time_total = time.time()
    
    if debug_mode:
        logger.info("Starting main control loop")
    
    # Main control loop
    while phase.value < 0.98:
        # Start timing this iteration to maintain consistent rate
        loop_start_time = time.time()
        
        # Get current policy value from DMP
        policy_value = policy.value.copy()  # Get the current value from the policy object
        
        # Update positions
        position_current[0], position_current[1] = get_ee_position(policy_value[0], policy_value[1], policy_value[2])
        
        # Get target trajectory point
        idx = traj_index * 3
        position_target[0], position_target[1] = get_ee_position(
            main_trajectory_ang[idx], 
            main_trajectory_ang[idx + 1], 
            main_trajectory_ang[idx + 2]
        )
        
        # Create state and get action from RL model
        state = [
            position_current[0], position_current[1],
            position_target[0], position_target[1],
            phase.value, x_ob, z_ob
        ]
        action = rl_actor.get_action(state)
        
        # Update phase and policy
        phase.update(ts)
        policy.update(ts, phase.value)
        
        # Check obstacle distance and apply avoidance if needed
        dist_to_obstacle = math.dist(position_current, obstacle_coordinates)
        if dist_to_obstacle < obstacle_check_threshold:
            # Apply RL action to the policy's value directly
            policy.value[0] += action[0]
            policy.value[1] += action[1]
            policy.value[2] += action[2]
            
        if debug_mode:
            logger.info(f"Obstacle avoidance activated! Distance: {dist_to_obstacle:.3f}, Action: {action}")
    
        if debug_mode:
            logger.info("=== SENDING GOAL TO CONTROLLER ===")
            logger.info(f"Current Phase: {phase.value}")
            logger.info(f"Joint angles (deg): {policy.value[0]:.4f}, {policy.value[1]:.4f}, {policy.value[2]:.4f}")
            logger.info(f"Joint angles (rad): {policy.value[0] * PI_OVER_180:.4f}, {policy.value[1] * PI_OVER_180:.4f}, {policy.value[2] * PI_OVER_180:.4f}")
            
        positions = [
            rad_neg180,
            policy.value[0] * PI_OVER_180, 
            rad_0, 
            rad_neg180 + policy.value[1] * PI_OVER_180,
            rad_0, 
            rad_135 + policy.value[2] * PI_OVER_180, 
            rad_0
        ]
        client.send_goal(*positions, duration=5.0)
        
        if debug_mode:
            logger.info("Goal has been sent using both methods")
            
        
        # Increment trajectory index
        traj_index += 1
        
        if debug_mode:
            elapsed_time = time.time() - loop_start_time
            if elapsed_time < ts:
                time.sleep(ts - elapsed_time)
            else:
                logger.warning(f"Loop iteration took {elapsed_time:.4f}s, which is longer than the desired {ts:.4f}s")
    
    elapsed_time = time.time() - loop_start_time_total
    logger.info(f"Total time: {elapsed_time:.4f}s")
    
    global phase_complete
    phase_complete = True

    subscriber.destroy_node()
    executor.shutdown()
    thread.join()
    main_ang_traj.close()
    # Signal that phase has reached 0.999
    
    logger.info("DONE")
    
    if debug_mode:
        logger.info("=== DMP EXECUTION COMPLETE ===")
        logger.info(f"Final trajectory index: {traj_index}")
        logger.info(f"Average time per iteration: {elapsed_time / traj_index:.6f}s")

# Initialize phase_complete flag
phase_complete = False

pkg_share = get_package_share_directory('manipulator')
urdf_path = pkg_share + '/resources/robot_description/manipulator.urdf'
robot = URDF.from_xml_file(urdf_path)
(_,kdl_tree) = kdl_parser.treeFromUrdfModel(robot)
kdl_chain = kdl_tree.getChain("panda_link0", "panda_finger")
fk_solver = kdl.ChainFkSolverPos_recursive(kdl_chain)

# Default paths
pkg_share = get_package_share_directory('manipulator_skill_acquisition')
default_mpx_path = pkg_share + '/resources/dmp/dmp.mpx'
default_rl_actor_path = pkg_share + '/resources/rl/EP.pt'

# Get paths from command line arguments if provided
if len(sys.argv) > 1:
    mpx_path = sys.argv[1]
else:
    mpx_path = default_mpx_path

if len(sys.argv) > 2:
    rl_actor_path = sys.argv[2]
else:
    rl_actor_path = default_rl_actor_path

# Check for debug flag
debug_mode = False
if len(sys.argv) > 3 and sys.argv[3] == "--debug":
    debug_mode = True
    logger.info("Debug mode enabled")

# load motion library from specified path
library = motion.mpx.load_from(mpx_path)
policy = library.policies[0]
phase = motion.LinearPacer(1.0)
tscaling = 1 / 2.5
phase.pace *= tscaling

# integration timestep
ts = 0.01

client = controller.ControllerClient(controller_type='auto')
client.set_debug_mode(debug_mode)

# Now initialize rl_actor AFTER rl_actor_path is defined
rl_actor = load_rl.RLActor(rl_actor_path)

# Only run trajectory controller test if debug mode is enabled
if debug_mode:
    logger.info("Debug mode: Running trajectory controller test")
    if test_trajectory_controller():
        logger.info("Trajectory controller test succeeded! Proceeding with DMP.")
    else:
        logger.warning("Trajectory controller test failed. DMP may not work correctly.")
else:
    logger.info("Debug mode disabled: Skipping trajectory controller test")

# Start the DMP thread
dmp_thread = Thread(target=dmp_load)
dmp_thread.start()

# Wait for DMP to complete
while not phase_complete:
    time.sleep(0.01)

# Clean up
client.destroy_node()
dmp_thread.join()
rclpy.shutdown()
