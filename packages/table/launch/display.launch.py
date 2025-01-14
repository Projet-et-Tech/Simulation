import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    
    pkg_name = 'table'
    file_subpath = 'description/table.urdf'
    
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
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    spawn_entity = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = [
            '-topic', 'table_description',
            '-entity', 'table'
        ],
        output = 'screen' 
    )
    
    return LaunchDescription([
        gazebo,
        spawn_entity
    ])