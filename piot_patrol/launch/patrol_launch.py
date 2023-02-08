#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='piot_patrol',
			executable='piot_patrol',
			name='piot_patrol_node',
			output='screen',
		)
	])
