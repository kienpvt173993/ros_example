from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the directory path of your package
    config_path = os.path.join(
        get_package_share_directory('search_example'),
        'param',
        'param.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='search_example',
            executable='search_example',
            name='search_example',
            output='screen',
            parameters=[config_path],
        ),
    ])
