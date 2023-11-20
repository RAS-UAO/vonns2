# Libraries for file handling
from ament_index_python import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Libraries for node launching
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

gazebo_pkg_share_filepath = get_package_share_directory("vonns2_bringup")

robot_launch_filepath = os.path.join(
    gazebo_pkg_share_filepath, 
    "launch", 
    "robot.launch.py"
)

reconstruction_pkg_share_filepath = get_package_share_directory("vonns2_reconstruction")
rviz_filename = "reconstruction_in_bringup.rviz"

rviz_filepath= os.path.join(
    reconstruction_pkg_share_filepath, 
    "config",
    rviz_filename
)

def generate_launch_description():
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_launch_filepath])
    )

    rviz_cmd = ExecuteProcess(
        cmd= ["rviz2", "-d", rviz_filepath]
    )

    laser_assembler_node = Node(
        package= "vonns2_reconstruction",
        executable= "laser_scan_assembler",
        output= "screen",
        emulate_tty=True,
        arguments=[("__log_level:=debug")]
    )
    
    nodes_to_run = [
        robot_launch,
        rviz_cmd,
        laser_assembler_node
    ]
    return LaunchDescription(nodes_to_run)