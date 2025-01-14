import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    
    pkg_name = 'table'
    file_subpath = 'description/table.urdf.xacro'
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),'launch'),
            '/gazebo.launch.py'
        ])
    )
    
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        file_subpath
    )
    table_description_raw = xacro.process_file(xacro_file).toxml()
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': table_description_raw,
            'use_sim_time': True
        }]
    )
    
    spawn_entity = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = [
            '-topic', 'robot_description',
            '-entity', 'table'
        ],
        output = 'screen' 
    )
    
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])