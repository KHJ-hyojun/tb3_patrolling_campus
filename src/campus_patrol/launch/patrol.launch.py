"""
patrol.launch.py - 순찰 시스템 전체 실행

노드 3개를 한 번에 띄운다.
Nav2와 Gazebo 는 별도로 실행해야 한다.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    patrol = Node(
        package='campus_patrol',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
    )

    logger = Node(
        package='campus_patrol',
        executable='logger_node',
        name='logger_node',
        output='screen',
    )

    monitor = Node(
        package='campus_patrol',
        executable='monitor_node',
        name='monitor_node',
        output='screen',
    )

    return LaunchDescription([patrol, logger, monitor])