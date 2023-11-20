from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name = "model", 
        default_value=os.path.join(get_package_share_directory("arduinobot_description"), "urdf", "arduinobot.urdf.xacro"),
        description="Absolute path to the robot URDF file."                              
    )

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("arduinobot_description"), "share"))

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": robot_description}]
    )

    # Set the path to the world file
    world_file_name = 'my_place.world'
    world_path = os.path.join(get_package_share_directory("arduinobot_description"), 'worlds', world_file_name)
    
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")),
        launch_arguments=[("world", world_path)]
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"))
    ) 

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "arduinobot", '-x', '0.0', '-y', '0.0', '-z', '0.0', '-R', '0.0', '-P', '0.0', '-Y', '0.0', "-topic", "robot_description"]
    )

    return LaunchDescription([env_var,
                              model_arg,
                              robot_state_publisher,
                              start_gazebo_server,
                              start_gazebo_client,
                              spawn_robot])