# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
# Libraries for node launching
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription

gazebo_pkg_share_filepath  = get_package_share_directory("vonns2_gazebo")

robot_state_publisher_launch_filepath = os.path.join(gazebo_pkg_share_filepath, "launch", "robot_state_publisher.launch.py")
urdf_filepath = os.path.join(gazebo_pkg_share_filepath, "description", "vonns2.urdf.xacro")
world_filepath = os.path.join(gazebo_pkg_share_filepath, "worlds", "cubic.sdf")

def generate_launch_description():
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_state_publisher_launch_filepath]),
        launch_arguments= {
            'use_sim_time': 'true'
        }.items()
    )
    
    gazebo_cmd = ExecuteProcess(
        cmd= [
            "gazebo", 
            "-s", "libgazebo_ros_factory.so",
            "--verbose"
        ],
        output= "screen",
    )

    gazebo_spawner_node = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'vonns2'
        ],
    )

    nodes_to_run = [
        robot_state_publisher_launch, 
        gazebo_cmd,
        gazebo_spawner_node
    ]
    return LaunchDescription(nodes_to_run)
       
