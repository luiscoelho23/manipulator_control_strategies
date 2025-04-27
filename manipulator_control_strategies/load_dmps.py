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
from sensor_msgs.msg import JointState

from threading import Thread
from manipulator import kdl_parser
import PyKDL as kdl
from urdf_parser_py.urdf import URDF

import subprocess
from ament_index_python.packages import get_package_share_directory

current_file = os.path.abspath(__file__)
if '/install/' in current_file:
    ws_path = current_file[:current_file.find('/install/')]
elif '/src/' in current_file:
    ws_path = current_file[:current_file.find('/src/')]
else:
    print("Error: Could not determine workspace path. Script must be run from install or src directory.")
    sys.exit(1)

mplibrary_path = os.path.join(ws_path, 'build/mplibrary')

sys.path.append(mplibrary_path)
import pymplibrary as motion

def get_ee_position(ang1,ang2,ang3):      
        
        joint_angles = kdl.JntArray(7)
        joint_angles[0] = (-180) * np.pi /180  # Joint 1 angle in radians
        joint_angles[1] = (ang1) * np.pi /180  # Joint 2 angle in radians
        joint_angles[2] = (0) * np.pi /180  # Joint 3 angle in radians
        joint_angles[3] = (-180 + ang2) * np.pi /180  # Joint 4 angle in radians
        joint_angles[4] = (0) * np.pi /180  # Joint 5 angle in radians
        joint_angles[5] = (135 + ang3) * np.pi /180  # Joint 6 angle in radians
        joint_angles[6] = 0 * np.pi /180  # Joint 7 angle in radians
        
        fk_solver = kdl.ChainFkSolverPos_recursive(kdl_chain)
        eeframe = kdl.Frame()
        fk_solver.JntToCart(joint_angles, eeframe)

        return eeframe.p.x() , eeframe.p.z()

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

                print(result.stdout)

        except subprocess.CalledProcessError as e:
            # Handle errors from the C++ program
            print("Error executing C++ program:", e.stderr)
            self.get_logger().error(f'Error in collision detection: {e.stderr}')
        except Exception as e:
            self.get_logger().error(f'Error in listener callback: {e}')


def dmp_load():
    
    main_traj = open("/home/luisc/main.csv", "w")
    real_traj = open("/home/luisc/rl.csv", "w")
    
    main_trajectory_ang = np.array([])

    x_ob, z_ob = -0.37, -0.25

    phase.value = 0.0
    policy.reset([117,120,31])

    while phase.value < 0.999: 
        phase.update(ts)
        policy.update(ts, phase.value)

        main_trajectory_ang = np.append(main_trajectory_ang,[policy.value[0], policy.value[1], policy.value[2]])
        x,z = get_ee_position(policy.value[0],policy.value[1],policy.value[2])
        out = str(x) + ";" + str(z) + "\n"
        main_traj.write(out)
        
    main_traj.close()
    
    print("DMP Loaded")

    phase.value = 0
    policy.reset([117,120,31])
    traj_index = 0

    
    # viapoints
    #policy.goal(0).addState(10, 0.5, 0.495)
    #policy.goal(1).addState(10, 0.5, 0.495)
    #policy.goal(2).addState(10, 0.5, 0.495)


    while phase.value < 0.999:
        x,z = get_ee_position(policy.value[0],policy.value[1],policy.value[2])
        x_t,z_t = get_ee_position(main_trajectory_ang[0 + traj_index * 3],main_trajectory_ang[1 + traj_index * 3],main_trajectory_ang[2 + traj_index * 3])
        state = [x,z,x_t,z_t,phase.value,x_ob,z_ob]
        action = rl_actor.get_action(state)
        print(action)
        phase.update(ts)
        policy.update(ts, phase.value)

        if math.dist([x,z],[x_ob,z_ob]) < 0.10:
            policy.value = [policy.value[0] + action[0] ,policy.value[1] + action[1] ,policy.value[2]+ action[2]]
        
        client.send_goal((-180) * np.pi / 180,( policy.value[0]  ) * np.pi / 180, 0, (-180 + policy.value[1]) * np.pi / 180,
                         0, (135 + policy.value[2]) * np.pi / 180, 0)
        out = str(x) + ";" + str(z) + "\n"
        real_traj.write(out)
        traj_index += 1
        time.sleep(0.001)
    
    print("DONE")
    real_traj.close()
    # Signal that phase has reached 0.999
    global phase_complete
    phase_complete = True

# Initialize phase_complete flag
phase_complete = False

pkg_share = get_package_share_directory('manipulator')
urdf_path = pkg_share + '/resources/robot_description/manipulator.urdf'
robot = URDF.from_xml_file(urdf_path)
(_,kdl_tree) = kdl_parser.treeFromUrdfModel(robot)
kdl_chain = kdl_tree.getChain("panda_link0", "panda_finger")

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

# load motion library from specified path
library = motion.mpx.load_from(mpx_path)
policy = library.policies[0]
phase = motion.LinearPacer(1.0)
tscaling = 1
phase.pace *= tscaling

# integration timestep
ts = 0.001

rclpy.init()

client = controller.ControllerClient(ts)
subscriber = DMPSubscriber()

# Create an executor for the subscriber
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(subscriber)

# Start the subscriber thread
thread = Thread(target=executor.spin, daemon=True)
thread.start()

# Initialize RL actor before starting the DMP
rl_actor = load_rl.RLActor(rl_actor_path)

# Start the DMP thread
dmp_thread = Thread(target=dmp_load)
dmp_thread.start()

# Wait for DMP to complete
while not phase_complete:
    time.sleep(0.1)

# Clean up
subscriber.destroy_node()
client.destroy_node()
executor.shutdown()
thread.join()
dmp_thread.join()
rclpy.shutdown()
