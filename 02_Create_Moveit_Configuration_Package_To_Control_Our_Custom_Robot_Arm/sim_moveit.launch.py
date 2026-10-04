# ---------------------------------------------------------
# IMPORT SECTION: Loading necessary ROS 2 and Python tools
# ---------------------------------------------------------
# The 'os' module provides operating system dependent functionality,
# specifically used here to safely build file paths across different OS environments.
import os
# LaunchDescription is the core container object in ROS 2 launch files.
# It holds the list of all nodes, processes, and included files that should be executed.
from launch import LaunchDescription
# IncludeLaunchDescription allows us to nest other launch files inside this one.
from launch.actions import IncludeLaunchDescription
# PythonLaunchDescriptionSource tells the launcher to interpret the included file as a Python launch script.
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Node is the primary class used to define and configure a single ROS 2 executable process.

from launch_ros.actions import Node
# This utility finds the absolute file path to the 'share' directory of an installed ROS 2 package,
# ensuring we don't have to hardcode absolute paths (like /home/user/workspace/...).
from ament_index_python.packages import get_package_share_directory
# MoveItConfigsBuilder is a powerful MoveIt 2 utility. It automatically parses the URDF, SRDF,
# kinematics, and joint limits to build a massive configuration dictionary needed by MoveIt.
from moveit_configs_utils import MoveItConfigsBuilder
# generate_launch_description() is the mandatory entry point for every ROS 2 launch file.
# The ROS 2 launch system automatically looks for and executes this specific function.
def generate_launch_description():
    # ---------------------------------------------------------
    # 1. GAZEBO INCLUSION: Starting the physics engine (Muscle)
    # ---------------------------------------------------------
    # Locate the install directory of the 'robot_arm_urdf' package.
    urdf_pkg = get_package_share_directory('robot_arm_urdf')
    # Create a launch action that executes the gazebo.launch.py file located inside the urdf package.
    # This brings up the Gazebo Harmonic simulator, loads the robot URDF, and starts the hardware controllers.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(urdf_pkg, 'launch', 'gazebo.launch.py')
        )
    )
    # ---------------------------------------------------------
    # 2. MOVEIT CONFIGURATION: Building the planner parameters
    # ---------------------------------------------------------
    # Initialize the builder by telling it the name of the URDF and the package containing the MoveIt config.
    moveit_config = (
        MoveItConfigsBuilder("robot_arm_urdf", package_name="test_moveit_package")
        # This specific line bypasses the strict ROS 2 YAML parser bug. It explicitly loads our
        # moveit_controllers.yaml file and parses it so MoveIt knows how to talk to Gazebo's controllers.
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # Finalizes the builder sequence and compiles all the parsed files into a configuration object.
        .to_moveit_configs()
    )
    # ---------------------------------------------------------
    # 3. CLOCK SYNCHRONIZATION: The Sim Time Fix
    # ---------------------------------------------------------
    # Convert the compiled MoveIt configuration object into a standard Python dictionary.
    moveit_params = moveit_config.to_dict()
    # Forcefully inject the 'use_sim_time': True parameter into the core dictionary.
    # This ensures that when MoveIt checks the time, it listens to Gazebo's clock (/clock topic)
    # rather than the computer's real-world hardware clock, preventing trajectory parameterization crashes.
    moveit_params.update({'use_sim_time': True})
    # ---------------------------------------------------------
    # 4. MOVE_GROUP NODE: Starting the Planning Brain
    # ---------------------------------------------------------
    # Define the primary MoveIt 2 node.
    move_group_node = Node(
        package="moveit_ros_move_group", # The ROS 2 package containing the executable.
        executable="move_group",
        # The actual C++ binary to run.
        output="screen",
        # Print the node's logs directly to the terminal.
        parameters=[moveit_params],
        # Feed the fully synced, controller-aware dictionary to
        # the node.
    )
    # ---------------------------------------------------------
    # 5. RVIZ NODE: Starting the User Interface
    # ---------------------------------------------------------
    # Build the safe file path to the default pre-configured RViz visual layout file.
    rviz_config_file = os.path.join(
        get_package_share_directory("test_moveit_package"),
        "config",
        "moveit.rviz",
    )
    # Define the RViz2 UI node.
    rviz_node = Node(
        package="rviz2",
        # The core ROS 2 visualization package.
        executable="rviz2",
        # The RViz2 binary.
        name="rviz2_moveit",
        # Assign a specific name to this node instance.
        output="log",
        # Send RViz's clutter logs to a file rather than spamming the terminal.
        arguments=["-d", rviz_config_file], # Launch RViz with the specific layout file we located
        # above.
        parameters=[moveit_params],
        # Pass the same MoveIt dictionary so RViz also uses
        # simulation time.
    )
    # ---------------------------------------------------------
    # 6. EXECUTION RETURN: Handing instructions to ROS 2
    # ---------------------------------------------------------
    # Return the final LaunchDescription object containing the list of actions.
    # ROS 2 will execute these concurrently: Gazebo, the MoveGroup planner, and RViz.
    return LaunchDescription([
        gazebo_launch,
        move_group_node,
        rviz_node
    ])