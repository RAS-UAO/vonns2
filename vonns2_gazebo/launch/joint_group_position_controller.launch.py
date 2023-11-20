# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
import xacro
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Libraries for node launching
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription


gazebo_pkg_share_filepath = get_package_share_directory("vonns2_gazebo")

robot_state_publisher_launch_filepath= os.path.join(
    gazebo_pkg_share_filepath, 
    "launch", 
    "robot_state_publisher.launch.py"
)

world_filename = "cylinder.sdf"
world_filepath = os.path.join(
    gazebo_pkg_share_filepath, 
    "worlds",
    world_filename
)

urdf_filepath = os.path.join(
    gazebo_pkg_share_filepath, 
    "description", 
    "vonns2.urdf.xacro"
)

robot_description_file = xacro.process_file(urdf_filepath)
robot_description = {"robot_description": robot_description_file.toxml()}

controller_filepath = os.path.join(gazebo_pkg_share_filepath, "config", "joint_group_position_controller.yaml")

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
            world_filepath,
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

    controller_manager_node = Node(
        package= "controller_manager",
        executable= "ros2_control_node",
        parameters= [robot_description, controller_filepath]
    )

    joint_state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments= ["joint_state_broadcaster"],
    )
    
    joint_group_position_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments= ["joint_group_position_controller"],
    )

    nodes_to_run = [
        robot_state_publisher_launch, 
        gazebo_cmd, 
        gazebo_spawner_node,
        controller_manager_node, 
        joint_state_broadcaster_spawner_node, 
        joint_group_position_controller_spawner_node
    ]
    return LaunchDescription(nodes_to_run)