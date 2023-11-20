# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
# Libraries for node launching
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription

bringup_pkg_share_filepath = get_package_share_directory("vonns2_bringup")
robot_state_publisher_launch_filepath= os.path.join(
    bringup_pkg_share_filepath, 
    "launch", 
    "robot_state_publisher.launch.py"
)
urg_port = "/dev/ttyACM0"
urg_yaml_filepath = "/opt/ros/humble/share/urg_node/launch/urg_node_serial.yaml"

def generate_launch_description():
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_state_publisher_launch_filepath]),
        launch_arguments= {
            'use_sim_time': 'true'
        }.items()
    )

    urg_allow_cmd = ExecuteProcess(
        cmd= ["sudo", "chmod", "a+rw", urg_port]
    )

    urg_driver_cmd = ExecuteProcess(
        cmd= [
            "ros2", "run", "urg_node", "urg_node_driver", 
            "--ros-args", "--params-file", urg_yaml_filepath
        ]
    )

    serial_port_reader_node = Node(
        package = "vonns2_bringup",
        executable = "serial_port_reader.py"
    )

    tf_broadcaster_node = Node(
        package = "vonns2_bringup",
        executable = "tf_broadcaster"
    )

    nodes_to_run = [
        robot_state_publisher_launch,
        urg_driver_cmd,
        urg_allow_cmd,
        serial_port_reader_node,
        tf_broadcaster_node,
    ]
    return LaunchDescription(nodes_to_run)
       
