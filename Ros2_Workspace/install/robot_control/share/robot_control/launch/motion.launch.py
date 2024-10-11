from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

	return LaunchDescription([
		launch_ros.actions.Node(
			package='robot_control',
			executable='motion_control.py',
			output='screen',
			arguments=["0.5","0.6", "1.2", "0.9", "0.7","1.6"]),
			
	])
