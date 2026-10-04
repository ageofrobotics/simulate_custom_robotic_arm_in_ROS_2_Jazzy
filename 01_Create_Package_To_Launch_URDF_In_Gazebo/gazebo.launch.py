# ==============================================================================
# STANDARD PYTHON & OPERATING SYSTEM MODULES
# ==============================================================================
# The 'os' module lets us interact with the operating system filesystem.
# We will use 'os.path.join' to construct portable file paths across Linux systems.
import os

# ==============================================================================
# ROS 2 CORE LAUNCH ENGINE IMPORTS
# ==============================================================================
# LaunchDescription: The primary container object that holds every action, node,
# and process ROS 2 must orchestrate.
from launch import LaunchDescription
# Action objects that perform system-level work rather than standard ROS 2 nodes:
from launch.actions import (
    # AppendEnvironmentVariable: Safely adds directories to environment variables (e.g., PATH)
    AppendEnvironmentVariable,
    # ExecuteProcess: Runs arbitrary shell/terminal commands (like running `ros2 control ...`)
    ExecuteProcess,
    # IncludeLaunchDescription: Allows nesting another launch file inside this one
    IncludeLaunchDescription,
    # RegisterEventHandler: Monitors life-cycle states or process completions to trigger actions
    RegisterEventHandler,
)
# OnProcessExit: An event condition that fires specifically when a target process finishes/exits
from launch.event_handlers import OnProcessExit
# PythonLaunchDescriptionSource: Informs the launch system that the included launch file is written in Python
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Command: A substitution tool that evaluates a shell command at runtime and captures standard output as a string
from launch.substitutions import Command

# ==============================================================================
# ROS 2 ROS-SPECIFIC ACTIONS & PACKAGE UTILITIES
# ==============================================================================
# Node: The launch action used specifically to start ROS 2 executable nodes
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # <--- ADD THIS LINE
# get_package_share_directory: Queries the ament resource index to locate where
# a package's installed files live (typically: install/<package_name>/share/<package_name>)
from ament_index_python.packages import get_package_share_directory

# ==============================================================================
# MAIN LAUNCH ENTRY POINT
# ==============================================================================
# Every Python ROS 2 launch file must expose a function named 'generate_launch_description()'
# that returns a populated LaunchDescription instance.
def generate_launch_description():
    # --------------------------------------------------------------------------
    # 1. RESOLVING PACKAGE PATHS & FILE LOCATIONS
    # --------------------------------------------------------------------------
    # The name of our custom robot package containing the URDF, meshes, and configs
    pkg_name = 'robot_arm_urdf'
    # Dynamically find the absolute path to this package in the install directory.
    # Never hardcode paths like '/home/user/...' so this code works on any computer!
    pkg_dir = get_package_share_directory(pkg_name)
    # Build the full absolute path to the main robot URDF / Xacro description file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot_arm_urdf.urdf')

    # --------------------------------------------------------------------------
    # 2. GAZEBO SIMULATION RESOURCE SEARCH PATH
    # --------------------------------------------------------------------------
    # In URDF files, 3D meshes are often referenced as 'package://robot_arm_urdf/meshes/...'.
    # Gazebo Sim does not read the ROS package index directly; it searches directories
    # listed in the 'GZ_SIM_RESOURCE_PATH' environment variable.
    #
    # We step up one directory ('..') to the parent 'share' directory so Gazebo can append
    # 'robot_arm_urdf/meshes/...' to that base directory and successfully find your STL/DAE files.
    append_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_dir, '..')
    )

    # --------------------------------------------------------------------------
    # 3. CONVERTING URDF / XACRO INTO RAW XML STRING
    # --------------------------------------------------------------------------
    # The Command substitution runs the shell command `xacro <path_to_urdf>` during launch.
    # It parses all macros, variables, and properties, outputting raw XML text.
    robot_desc = Command(['xacro ', urdf_file])

    # --------------------------------------------------------------------------
    # 4. ROBOT STATE PUBLISHER NODE
    # --------------------------------------------------------------------------
    # robot_state_publisher does two critical jobs:
    # 1. Takes the raw XML from 'robot_description' and publishes it onto the /robot_description topic.
    # 2. Reads joint positions from the /joint_states topic and calculates 3D coordinate frame transforms (TF).
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',  # Stream logs directly to the launch console terminal
        parameters=[{
            'robot_description': ParameterValue(robot_desc, value_type=str),
            # use_sim_time=True forces this node to sync its timestamps with the Gazebo /clock topic
            # rather than the computer's internal wall clock.
            'use_sim_time': True
        }]
    )

    # --------------------------------------------------------------------------
    # 5. STARTING THE GAZEBO SIMULATOR
    # --------------------------------------------------------------------------
    # Locate the official 'ros_gz_sim' ROS wrapper package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Include the pre-packaged Gazebo launch file provided by ros_gz_sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # 'empty.sdf': Loads a world with basic lighting and a ground plane.
        # '-r': Runs physics immediately on startup without needing to press the Play button in the GUI.
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    # --------------------------------------------------------------------------
    # 6. SPAWNING THE ROBOT INTO GAZEBO
    # --------------------------------------------------------------------------
    # The 'create' executable in ros_gz_sim reads our parsed URDF from the /robot_description
    # ROS topic and injects it as an active physical model into Gazebo under the name 'robot_arm'.
    # IMPORTANT: This executable terminates and exits as soon as the robot finishes spawning.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robot_arm'],
        output='screen'
    )

    # --------------------------------------------------------------------------
    # 7. ROS 2 <-> GAZEBO CLOCK BRIDGE
    # --------------------------------------------------------------------------
    # Gazebo and ROS 2 are separate systems communicating over middleware bridges.
    # This bridge forwards the simulation clock from Gazebo ('gz.msgs.Clock') to ROS 2 ('rosgraph_msgs/msg/Clock').
    # Without this, all nodes set to 'use_sim_time: True' will freeze waiting for time ticks.
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # --------------------------------------------------------------------------
    # 8. CONTROLLER COMMAND DEFINITIONS
    # --------------------------------------------------------------------------
    # ExecuteProcess runs the CLI command `ros2 control load_controller --set-state active <controller_name>`.
    # These interact with the Gazebo ros2_control plugin, which hosts the Controller Manager.
    # Joint State Broadcaster: Publishes the current positions/velocities of all joints to /joint_states
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    # Arm Controller: Controls trajectories and positions for arm joints (joint_1 to joint_5)
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    
    # Gripper Controller: Controls the gripping motion (joint_6 and joint_7)
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )

    # --------------------------------------------------------------------------
    # 9. SEQUENTIAL EVENT HANDLERS (ELIMINATING RACE CONDITIONS)
    # --------------------------------------------------------------------------
    # If we load controllers immediately at startup, the Controller Manager inside Gazebo
    # may not have finished initializing, causing "Service not available" failures.
    # We daisy-chain the loading process using OnProcessExit:

    # Step A: Wait until the robot model is fully spawned into Gazebo before loading Joint State Broadcaster
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )
    
    # Step B: Wait until the Joint State Broadcaster is active before activating the Arm Controller
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller],
        )
    )
    
    # Step C: Wait until the Arm Controller is active before activating the Gripper Controller
    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller,
            on_exit=[load_gripper_controller],
        )
    )

    # --------------------------------------------------------------------------
    # 10. RETURN THE LAUNCH DESCRIPTION
    # --------------------------------------------------------------------------
    # We pass all primary actions to LaunchDescription.
    # NOTE: We include the EVENT HANDLERS (delay_*), NOT the raw controller actions (load_*).
    # The event handlers will trigger the load commands at the appropriate moments in time.
    return LaunchDescription([
        append_resource_path,
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge_clock,
        delay_joint_state_broadcaster,
        delay_arm_controller,
        delay_gripper_controller
    ])
