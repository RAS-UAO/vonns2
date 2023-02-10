from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription

pkg_path = get_package_share_directory('vonns2')

def generate_launch_description():
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'vonns2.urdf')
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file_path]
    )
    
    gazebo_cmd = ExecuteProcess(
    	cmd= ['gazebo', '-s', "libgazebo_ros_factory.so", "-s", "libgazebo_ros_init.so"]
    )

    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments= ["-topic", "robot_description", "-entity", "vonns2"]
    )

    return LaunchDescription(
        [
            rsp_node,
            gazebo_cmd,
            spawn_node
        ]
    )
