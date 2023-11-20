# Libraries for file handling
from ament_index_python import get_package_share_directory
import os
# Libraries for node launching
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription

gazebo_pkg_share_filepath = get_package_share_directory("vonns2_gazebo")

robot_state_publisher_filepath = os.path.join(
    gazebo_pkg_share_filepath, 
    "launch", 
    "robot_state_publisher.launch.py"
)

visualization_filepath = os.path.join(
    gazebo_pkg_share_filepath, 
    "config", 
    "visualization.rviz"
)

def generate_launch_description():
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_state_publisher_filepath]),
        launch_arguments= {
            "use_sim_time": "true"
        }.items()
    )

    rviz_cmd = ExecuteProcess(
        cmd= ["rviz2", "-d", visualization_filepath]
    )

    jsp_gui_node = Node(
        package= "joint_state_publisher_gui",
        executable= "joint_state_publisher_gui",
    )

    nodes_to_run = [   
        robot_state_publisher_launch,
        rviz_cmd,
        jsp_gui_node
    ]
    return LaunchDescription(nodes_to_run)