from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchContext

def generate_launch_description():
	pkg_share = get_package_share_directory('my_python_pkg')
	script_path = os.path.join(pkg_share, 'my_python_pkg', 'my_python_node.py')
	  
	return LaunchDescription([
		DeclareLaunchArgument(
			'num_turtles',
			default_value='2',  # Default number of turtles
			description='Number of turtles to create'
		),
		Node(
			package='turtlesim',
			executable='turtlesim_node',
			output='screen',
			emulate_tty=True,
			parameters = [{'background_r':80, 'background_g':0, 'background_b':0}]
		),
		Node(
			package='my_python_pkg',
			executable='my_python_node',
			#name='my_python_node',
			output='screen',
			parameters=[{'num_turtles': LaunchConfiguration('num_turtles')}],
		)
	])

	