#!/usr/bin/python3

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Starts turtlesim_plus_node and pizza_on_click.py together, replacing the
two-terminal `ros2 run` workflow from the README. Scanner/eat tuning is exposed
as launch arguments so it doesn't require editing source."""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    time_step_arg = DeclareLaunchArgument('time_step', default_value='0.01')
    scanner_radius_arg = DeclareLaunchArgument('scanner_radius', default_value='4.0')
    scanner_angle_range_arg = DeclareLaunchArgument('scanner_angle_range', default_value=str(math.pi / 3))
    eat_radius_arg = DeclareLaunchArgument('eat_radius', default_value='2.0')
    eat_angle_range_arg = DeclareLaunchArgument('eat_angle_range', default_value=str(math.pi / 3))

    turtlesim_plus_node = Node(
        package='turtlesim_plus',
        executable='turtlesim_plus_node.py',
        name='turtlesim_plus',
        output='screen',
        parameters=[{
            'time_step': LaunchConfiguration('time_step'),
            'scanner_radius': LaunchConfiguration('scanner_radius'),
            'scanner_angle_range': LaunchConfiguration('scanner_angle_range'),
            'eat_radius': LaunchConfiguration('eat_radius'),
            'eat_angle_range': LaunchConfiguration('eat_angle_range'),
        }],
    )

    pizza_on_click_node = Node(
        package='turtlesim_plus',
        executable='pizza_on_click.py',
        name='spawn_pizza_on_click',
        output='screen',
    )

    return LaunchDescription([
        time_step_arg,
        scanner_radius_arg,
        scanner_angle_range_arg,
        eat_radius_arg,
        eat_angle_range_arg,
        turtlesim_plus_node,
        pizza_on_click_node,
    ])
