from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('map_generator')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'test.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        Node(
            package='map_generator',
            executable='map_generator_node',
            name='map_generator_node',
            output='screen',  # none， screen，log 三种输出方式
            parameters=[{'grid_map_topic': 'grid_map'}]  # 设置话题名称
        ),
    ])
