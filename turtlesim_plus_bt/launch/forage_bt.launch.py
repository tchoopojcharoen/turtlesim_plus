#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    turtle_name_arg = DeclareLaunchArgument('turtle_name', default_value='turtle1')

    forage_bt_node = Node(
        package='turtlesim_plus_bt',
        executable='forage_bt_node',
        name='forage_bt',
        output='screen',
        parameters=[{'turtle_name': LaunchConfiguration('turtle_name')}],
    )

    return LaunchDescription([turtle_name_arg, forage_bt_node])
