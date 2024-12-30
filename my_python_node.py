import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist 
import math
import cv2
import numpy as np
from turtlesim.msg import Pose 
from rclpy.executors import MultiThreadedExecutor
import time
from math import pi
from time import sleep
import collections
from std_srvs.srv import Empty

lines = [
# T
[(1.5, 6.3, pi/2), (1.5, 8.3, 0.0)], 
[(1.5, 8.3, 0.0), (8.5, 8.3, -pi/2)],
[(8.5, 8.3, -pi/2), (8.5, 6.3, pi)],
[(8.5, 6.3, pi), (7.1, 6.3, pi/2)], 
[(7.1, 6.3, pi/2), (7.1, 7.0, pi)], 
[(7.1, 7.0, pi), (5.7, 7.0, -pi/2)],
[(3.5, 1.0, pi/2), (3.5, 2.4, 0.0)],
[(3.5, 2.4, 0.0), (4.5, 2.4, pi/2)],
[(5.7, 7.0, -pi/2), (5.7, 2.4, 0.0)], 
[(5.7, 2.4, 0.0), (6.4, 2.4, -pi/2)],
[(6.4, 2.4, -pi/2), (6.4, 1.0, pi)],
[(6.4, 1.0, pi), (3.5, 1.0, 0.0)],
[(4.5, 2.4, pi/2), (4.5, 7.0, pi)],
[(4.5, 7.0, pi), (2.9, 7.0, -pi/2)],
[(2.9, 7.0, -pi/2), (2.9, 6.3, pi)], 
[(2.9, 6.3, pi), (1.5, 6.3, pi/2)],
[(1.7, 3.4, -pi/2),(1.7, 2.7, pi)],
[(1.7, 2.7, pi),(0.5, 2.7, pi/2)],
# A
[(0.5, 2.7, pi/2),(0.5, 3.4, 0.0)],
[(0.5, 3.4, 0.0),(0.8, 3.4, 1.16)], 
[(0.8, 3.4, 1.16),(1.7, 5.4, pi)],
[(1.7, 5.4, pi),(1.5, 5.4, pi/2)],
[(1.7, 3.7, -1.97),(1.5, 3.4, 0.0)], 
[(1.5, 3.4, 0.0),(1.7, 3.4, -pi/2)],
[(1.5, 5.4, pi/2),(1.5, 5.9, 0.0)],
[(1.5, 5.9, 0.0),(2.9, 5.9, -pi/2)],
[(2.9, 5.9, -pi/2),(2.9, 5.4, pi)],
[(2.9, 5.4, pi),(2.7, 5.4, -1.16)], 
[(2.7, 5.4, -1.16),(3.5, 3.4, 0.0)],
[(3.5, 3.4, 0.0),(3.9, 3.4, -pi/2)],
[(3.9, 3.4, -pi/2),(3.9, 2.7, pi)],
[(3.9, 2.7, pi),(2.6, 2.7, pi/2)],
[(2.6, 2.7, pi/2),(2.6, 3.4, 0.0)],
[(2.6, 3.4, 0.0),(2.8, 3.4, 1.97 )], 
[(2.8, 3.4, 1.97),(2.6, 3.7, pi)],
[(2.6, 3.7, pi),(1.7, 3.7, 1.64)], 
[(2.0, 4.3, 1.14), (2.3, 4.7, -1.14)],
[(2.3, 4.7, -1.14), (2.5, 4.3, pi)],
[(2.5, 4.3, pi), (2.0, 4.3, -1.97)],
# M
[[7.0, 3.4, 0.0],[7.3, 3.4, -pi/2]],
[[7.3, 3.4, -pi/2],[7.3, 2.7, pi]],
[[7.3, 2.7, pi],[6.1, 2.7, pi/2]],
[[6.1, 2.7, pi/2],[6.1, 3.4, 0.0]],
[[6.1, 3.4, 0.0],[6.4, 3.4, pi/2]],
[[6.4, 3.4, pi/2],[6.4, 5.4, pi]],
[[6.4, 5.4, pi],[6.1, 5.4, pi/2]],
[[6.1, 5.4, pi/2],[6.1, 5.9, 0.0]],
[[6.1, 5.9, 0.0],[7.2, 5.9, -pi/3]],
[[7.2, 5.9, -pi/3],[7.6, 4.5, pi/3]],
[[7.7, 4.5, pi/3],[8.4, 5.9, 0.0]],
[[8.4, 5.9, 0.0],[9.5, 5.9, -pi/2]],
[[9.5, 5.9, -pi/2],[9.5, 5.4, pi]],
[[9.5, 5.4, pi],[9.2, 5.4, -pi/2]],
[[9.2, 5.4, -pi/2],[9.2, 3.4, 0.0]],
[[8.3, 2.7, pi/2],[8.3, 3.4, 0.0]],
[[8.3, 3.4, 0.0],[8.6, 3.4, pi/2]],
[[8.6, 3.4, pi/2],[8.6, 4.7, -2*pi/3]],
[[8.6, 4.7, -2*pi/3],[7.7, 3.1, 2*pi/3]],
[[7.7, 3.1, 2*pi/3],[7.0, 4.7, -pi/2]],
[[7.0, 4.7, -pi/2],[7.0, 3.4, 0.0]],
[[9.2, 3.4, 0.0],[9.5, 3.4, -pi/2]],
[[9.5, 3.4, -pi/2],[9.5, 2.7, pi]],
[[9.5, 2.7, pi],[8.3, 2.7, pi/2]],
]

# Define a class called LogoDrawer that inherits from the Node class
class LogoDrawer(Node):
	def __init__(self, num_turtles=1):
		# Call the constructor of the parent class and set the node name
		super().__init__('my_python_node')
		# Declare a parameter 'num_turtles' with a default value of 1
		self.declare_parameter('num_turtles', 1)
        # Retrieve the value of 'num_turtles' parameter
		self.num_turtles = self.get_parameter('num_turtles').value

        # Initialize lists and dictionaries to manage turtles and lines
		self.turtle_names = []
		self.turtles = collections.defaultdict(TurtleController)
		self.initialized = False

         # Divide lines into parts based on the number of turtles
		self.divided_lines = self.divide_lines_into_parts(lines, self.num_turtles)
		
        # Perform initial cleanup to kill any existing turtles
		self.initial_cleanup()

        # If the initialization was successful, create turtles and assign lines
        if self.initialized == False:
		if self.initialized == False:
			for i in range(self.num_turtles):
				turtle_name = f'turtle{i}'
				self.turtles[turtle_name] = TurtleController(turtle_name, self.divided_lines[i])
			self.initialized = True

	def initial_cleanup(self):
        # Create a client to call the 'kill' service
		kill_client = self.create_client(Kill, 'kill')

        # Wait for the 'kill' service to become available
		while not kill_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')
        # Create a request to kill a turtle
		kill_request = Kill.Request()
		kill_request.name = 'turtle1'

        # Call the 'kill' service asynchronously
		kill_client.call_async(kill_request)

	def line_length(self, line):
		x1, y1, _ = line[0]
		x2, y2, _ = line[1]
        # Calculate the length of a line segment
		return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    # Define a function called 'divide_lines_into_parts' that takes 'lines' and 'n' as parameters
	def divide_lines_into_parts(self, lines, n):
        # Check if the number of turtles requested is greater than the number of lines
		if n > len(lines):
            # Print an error message and return if the condition is not met
			print('invalid. please keep number of turtles atmost number of lines ')
			return
        # Calculate how many lines each part should have in equal division
		equal_division = len(lines)//n
        # Initialize variables to keep track of the current part and parts
		current_part = 0
		parts = [[] for _ in range(n)]

        # Iterate through each line
		for line in lines:
            # Check if the current part has fewer lines than the equal division
			if len(parts[current_part]) < equal_division:
                # Add the line to the current part
				parts[current_part].append(line)
			else:
                # If the current part is not the last part, switch to the next part
				if current_part + 1 < n:
					current_part += 1
                # Add the line to the new current part
				parts[current_part].append(line)
		# Return the list of parts containing divided lines
		return parts


# Define a new class called 'TurtleController' that inherits from the Node class
class TurtleController(Node):
	# constructor method
	def __init__(self, number, lines_to_be_drawn):
		# initialize node with unique name
		super().__init__(f'my_python_node_{number}')
		
        # Set the turtle's name based on the 'number' parameter
		self.turtle_name = f'turtle{number}'
		self.get_logger().info(self.turtle_name)
		
		# Create a publisher to send Twist type messages to the turtle's command velocity topic
		self.cmd_publisher = self.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel', 1)
		
		# Create a subscriber to receive pose updates from the turtle
		self.pose_subscriber = self.create_subscription(Pose, f'/{self.turtle_name}/pose', self.pose_callback, 1)
		
        # Create a client to interact with the 'set_pen' service
		self.pen_client = self.create_client(SetPen, f'/{self.turtle_name}/set_pen')
		
        # Spawn the turtle in the simulation
		self.spawn_turtle(self.turtle_name)		
		
		# Create a timer to call the 'timer_callback' method every 0.1 seconds
		self.timer = self.create_timer(0.1, self.timer_callback)
		self.pose = Pose()
		self.current_index = 0
		self.goals = lines_to_be_drawn
		# self.goals.append(self.goals[-1])
		self.get_logger().info(f'{self.turtle_name} : {lines_to_be_drawn}')

        # Initialize variables to manage turtle's state and goals
		self.start_pose = Pose()
		self.goal_pose = Pose()

		self.reached_start = False
		self.reached_goal = False
		self.started = False
		

	def spawn_turtle(self, name):
        # Create a client to interact with the 'spawn' service
		spawn_client = self.create_client(Spawn, 'spawn')

        # Wait for the 'spawn' service to become available
		while not spawn_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')
        # Create a request to spawn the turtle at specific coordinates
		spawn_request = Spawn.Request()
		spawn_request.name = self.turtle_name
		spawn_request.x = 5.544445
		spawn_request.y = 5.544445

        # Call the 'spawn' service asynchronously to spawn the turtle
		spawn_client.call_async(spawn_request)

        # Create a request to set the pen off initially
		request = SetPen.Request()
		request.off = 1

        # Call the 'set_pen' service asynchronously to turn off the turtle's pen
		self.pen_client.call_async(request)

	def kill_turtle(self, name):
        # Create a client to interact with the 'kill' service
		kill_client = self.create_client(Kill, 'kill')

        # Wait for the 'kill' service to become available
		while not kill_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')

        # Create a request to kill the specified turtle
		kill_request = Kill.Request()
		kill_request.name = f'{name}'

        # Call the 'kill' service asynchronously to kill the turtle
		kill_client.call_async(kill_request)
	
    # Define a callback method to update the turtle's pose when a new pose message is received
	def pose_callback(self, msg):
		self.pose = msg
	
    # Define a callback method called 'timer_callback' that gets executed by a timer
	def timer_callback(self):
        # Check if there are more goals to reach
		if self.current_index < len(self.goals):
			if not self.started:
                # Set the pen off initially
				request = SetPen.Request()
				request.off = 1
				self.pen_client.call_async(request)

                # Store the start and goal pose for the current goal
				self.start_pose.x = self.goals[self.current_index][0][0]
				self.start_pose.y = self.goals[self.current_index][0][1]
				self.start_pose.theta = self.goals[self.current_index][0][2]
				self.goal_pose.x = self.goals[self.current_index][1][0]
				self.goal_pose.y = self.goals[self.current_index][1][1]
				self.goal_pose.theta = self.goals[self.current_index][1][2]
				# self.get_logger().info(str(f'{self.start_pose} \n {self.goal_pose} \n {self.pose}'))

                # Calculate the angle to turn to face the start position
				current_angle = self.pose.theta

                # Ensure angle is within the range [-pi, pi]
				go_to_start_angle = round(math.atan2(self.start_pose.y - self.pose.y, self.start_pose.x - self.pose.x), 3) - current_angle
				if abs(go_to_start_angle) > 0.001:
					go_to_start_angle = (go_to_start_angle + math.pi) % (2 * math.pi) - math.pi

                    # Turn the turtle to face the start position
					self.turn(go_to_start_angle)
					self.get_logger().info(str(f'{go_to_start_angle}'))
					sleep(1)

                # Calculate the distance to move to reach the start position
				go_to_start_distance = round(math.sqrt(math.pow((self.start_pose.x - self.pose.x), 2) + pow((self.start_pose.y - self.pose.y), 2)), 2)

                # Move the turtle to the start position
				if abs(go_to_start_distance) >= 0.01:
					self.move(go_to_start_distance)
					self.get_logger().info(str(f'{go_to_start_distance}'))
					sleep(abs(go_to_start_distance))

				# self.get_logger().info(str(f'{self.start_pose} \n {self.goal_pose} \n {self.pose}'))
				self.started = True

			if not self.reached_start:
				# self.get_logger().info(str(f'inside start if'))
                # Calculate the angle error to face the start pose
				angle_diff = self.start_pose.theta - self.pose.theta
				angle_error = round(math.atan2(math.sin(angle_diff), math.cos(angle_diff)), 3)

                # Ensure angle error is within the range [-pi, pi]
				if abs(angle_error) > 0.001:
					angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

                    # Turn the turtle to face the start pose
					self.turn(angle_error)
				#	self.get_logger().info(str(f'turning by {angle_error}'))
					sleep(0.01)# sleep(0.01)
					return
				self.reached_start = True
				self.move(0.0)
				self.turn(0.0)
				
				return
			if not self.reached_goal:
                # Set the pen to draw
				request = SetPen.Request()
				request.off = 0
				request.r = 255
				request.g = 255
				request.b = 255
				self.pen_client.call_async(request)

                # Calculate the distance error to reach the goal pose
				distance_error = round(math.sqrt(math.pow((self.goal_pose.x - self.pose.x), 2) + pow((self.goal_pose.y - self.pose.y), 2)), 2)

                # Move the turtle towards the goal pose
				if abs(distance_error) >= 0.1: # 0.1
					self.move(distance_error)
				#	self.get_logger().info(str(f'moving by {distance_error}'))
					sleep(1) # no sleep
					#return


				self.move(0.0)
				self.turn(0.0)
				self.reached_goal = True
				#self.get_logger().info(str(f'reached goal pose'))
				#self.get_logger().info(str(f'{self.start_pose} \n {self.goal_pose} \n {self.pose}'))
				return
			
            # Reset state variables for the next goal
			self.reached_goal = False
			self.reached_start = False
			self.started = False

            # Set the pen off
			request = SetPen.Request()
			request.off = 1
			self.pen_client.call_async(request)

            # Move to the next goal in the list
			self.current_index += 1		
						
			self.get_logger().info(str(f'{self.current_index}: {self.start_pose} \n {self.goal_pose} \n {self.pose}'))
			return
				
		else:
            # Cancel the timer and kill the turtle when all goals are reached
			self.timer.cancel()
			self.kill_turtle(self.turtle_name)
			
	# Define a method called 'move' to publish linear velocity commands	
	def move(self, data):
		twist = Twist()
		twist.linear.x = data
		twist.angular.z = 0.0
		self.cmd_publisher.publish(twist)

    # Define a method called 'turn' to publish angular velocity commands
	def turn(self, data):
		twist = Twist()
		twist.linear.x = 0.0
		twist.angular.z = data
		self.cmd_publisher.publish(twist)

# starting point of script
def main(args=None):
	# initialize rclpy library
	rclpy.init(args=args)
	executor = MultiThreadedExecutor()
	logo_drawer = LogoDrawer()
	executor.add_node(logo_drawer)	
	for turtle_controller in logo_drawer.turtles.values():
		executor.add_node(turtle_controller)
	executor.spin()
	
	# cleanup after program is finished
	executor.destroy_node()
	
	# release resources and shutdown
	rclpy.shutdown()

	executor.destroy_node()
	rclpy.shutdown()

# Execute the 'main' function if this script is run as the main module
if __name__ == '__main__':
	main()
	
	