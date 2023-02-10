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
    
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        arguments= [urdf_file_path]
    )
    
    rviz_file_path = os.path.join(pkg_path, 'rviz', 'urdf.rviz')
    rviz_cmd = ExecuteProcess(
    	cmd= ['rviz2', "-d", rviz_file_path]
    )

    return LaunchDescription(
        [
            jsp_gui_node,
            rsp_node,
            rviz_cmd
        ]
    )
