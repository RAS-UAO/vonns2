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

gazebo_pkg_share_filepath = get_package_share_directory("vonns2_gazebo")

joint_group_position_controller_launch_filepath = os.path.join(
    gazebo_pkg_share_filepath, 
    "launch", 
    "joint_group_position_controller.launch.py"
)

reconstruction_pkg_share_filepath = get_package_share_directory("vonns2_reconstruction")
rviz_filename = "reconstruction_in_gazebo.rviz"

rviz_filepath= os.path.join(
    reconstruction_pkg_share_filepath, 
    "config",
    rviz_filename
)

def generate_launch_description():
    joint_group_position_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([joint_group_position_controller_launch_filepath])
    )

    joint_group_position_publisher_node = Node(
        package= "vonns2_gazebo",
        executable= "joint_group_position_publisher.py"
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
        joint_group_position_controller_launch,
        joint_group_position_publisher_node,
        rviz_cmd,
        laser_assembler_node
    ]
    return LaunchDescription(nodes_to_run)