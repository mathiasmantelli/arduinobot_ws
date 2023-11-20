from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(
        get_package_share_directory("arduinobot_description"), 
        "launch",
        "gazebo.launch.py"
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
        get_package_share_directory("arduinobot_controller"), 
        "launch",
        "controller.launch.py"
        )
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
        get_package_share_directory("arduinobot_moveit"), 
        "launch",
        "moveit.launch.py"
        )
    )

    remote_interface = IncludeLaunchDescription(
        os.path.join(
        get_package_share_directory("arduinobot_remote"), 
        "launch",
        "remote_interface.launch.py"
        )
    )


    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        remote_interface
    ])