from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription

pkg_path = get_package_share_directory('vonns2')

def generate_launch_description():
    #sudo chmod a+rw /dev/ttyACM0
    urg_file_path = "/opt/ros/foxy/share/urg_node/launch/urg_node_serial.yaml"
    urg_node = Node(
        package= "urg_node",
        executable= "urg_node_driver",
        parameters= [urg_file_path]
    )

    urdf_file_path = os.path.join(pkg_path, 'urdf', 'vonns2.urdf')
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file_path]
    )
    
    tf_broadcaster_node = Node(
        package= "vonns2",
        executable= "tf_broadcaster.py"
    )
    
    rviz_file_path = os.path.join(pkg_path, 'rviz', 'reconstruction3d.rviz')
    rviz_cmd = ExecuteProcess(
    	cmd= ['rviz2', '-d', rviz_file_path]
    )

    return LaunchDescription(
        [
            urg_node,
            rsp_node,
            tf_broadcaster_node,
            rviz_cmd
        ]
    )
