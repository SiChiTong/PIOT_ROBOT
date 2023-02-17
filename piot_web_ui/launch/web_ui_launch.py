#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='piot_web_ui',
			executable='piot_web_ui',
			name='piot_web_ui_node',
			output='screen',
		)
	])
