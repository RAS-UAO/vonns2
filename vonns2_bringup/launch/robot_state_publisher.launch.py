# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
import xacro
# Libraries for node launching
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

gazebo_pkg_share_filepath = get_package_share_directory("vonns2_gazebo")

urdf_filepath = os.path.join(
    gazebo_pkg_share_filepath, 
    "description", 
    "vonns2.urdf.xacro"
)
robot_description_config = xacro.process_file(urdf_filepath)

use_sim_time = LaunchConfiguration('use_sim_time')

def generate_launch_description():
    use_sim_time_declaration = DeclareLaunchArgument(
        'use_sim_time', 
        default_value= 'true', 
        description= 'Use sim time if true'
    )

    robot_state_publisher_node = Node(
        package= "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_config.toxml(),
                'use_sim_time': use_sim_time
            }
        ]
    )

    nodes_to_run = [
        use_sim_time_declaration, 
        robot_state_publisher_node
    ]
    return LaunchDescription(nodes_to_run)
