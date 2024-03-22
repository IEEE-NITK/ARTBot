import rclpy  # Import the ROS Client Library for Python
from rclpy.node import Node  # Import the Node class from ROS
from geometry_msgs.msg import Twist  # Import the Twist message type for publishing velocity commands
from artbotsim.msg import Pose  # Import the Pose message type for subscribing to robot pose
from artbotsim.msg import Target  # Import the Target message type for subscribing to target positions
import math  # Import the math module for mathematical functions
from math import atan2, sqrt  # Import the atan2 and sqrt functions from the math module
from math import pi as p  # Import pi from the math module and alias it as 'p'
import re  # Import the regular expression module

bots = 50  # Number of bots
class ArtistBot(Node):
    def __init__(self, bot_name, target_coordinates_list):
        super().__init__(bot_name)  # Initialize the Node with the bot_name
        self.bot_name = bot_name  # Store the bot name
        self.target_coordinates = target_coordinates_list  # Store the list of target coordinates
        self.target = target_coordinates_list  # Create a copy of the target coordinates list
        self.current = [0.0, 0.0, 0.0]  # Initialize current position to (0, 0, 0)
        self.i = 0  # Initialize the bot index to 0
        self.letter = 1  # Initialize the letter index to 1 (not used in this code)
        self.target_subscriber = self.create_subscription(Target, 'target', self.target_callback, 10)  # Create a subscription to the 'target' topic
        self.pose_subscribers = self.create_subscription(Pose, f'{bot_name}/pose', self.pose_callback, 10)  # Create a subscription to the bot's pose topic
        self.velocity_publisher = self.create_publisher(Twist, f'{bot_name}/cmd_vel', 10)  # Create a publisher for the bot's velocity commands
        self.integral_error_x = 0.0  # Initialize the integral error for linear velocity to 0
        self.integral_error_y = 0.0  # Initialize the integral error for angular velocity to 0
        self.prev_error_x = 0.0  # Initialize the previous linear error to 0
        self.prev_error_y = 0.0  # Initialize the previous angular error to 0

        self.timer = self.create_timer(7.0, self.update_targets)  # Create a timer that calls the update_targets function every 7 seconds
        
    def target_callback(self, msg):
        # Update the target positions for odd-numbered bots
        for i in range(1, bots):  # Loop through bot numbers 1 to 30
            bot_name = f'artist{i}'  # Construct the bot name
            if (i % 2) != 0:  # Check if the bot number is odd
                x = getattr(msg, f'target{i}_x')  # Get the x-coordinate of the target from the message
                y = 810 - getattr(msg, f'target{i}_y')  # Get the y-coordinate of the target from the message (720 - y to account for coordinate system)
                self.target[bot_name] = [x/45, y/45]  # Update the target position for the bot (scale the coordinates)
            else:
                self.target[bot_name] = None  # Set the target to None for even-numbered bots
		    
    def pose_callback(self, msg):
        bot_name = msg.src  # Get the bot name from the message source
        characters_to_remove = "[artist]"  # Define a pattern to remove from the bot name
        self.i = int(re.sub(characters_to_remove, "", bot_name))  # Extract the bot number from the name using regex substitution
        self.current[0] = msg.x  # Update the current x-coordinate from the message
        self.current[1] = msg.y  # Update the current y-coordinate from the message
        self.current[2] = msg.theta  # Update the current orientation from the message
        if self.i==0:    
            print(self.current)
        self.calculate_and_publish_velocity(bot_name)  # Call the function to calculate and publish the velocity

    def update_targets(self):
        self.target_coordinates = self.target  # Update the target coordinates list

    def calculate_and_publish_velocity(self, bot_name):
        current_position = self.current  # Get the current position of the bot
        angular_velocity = 0.0
        linear_velocity = 0.0
        error_threshold = 0.015  # Error threshold for stopping at the target
                # PID controller gains
        kp_linear = 0.28  # Proportional gain for linear velocity
        ki_linear = 0.000  # Integral gain for linear velocity
        kd_linear = 0.0008  # Derivative gain for linear velocity
        kp_angular = 2.35  # Proportional gain for angular velocity
        ki_angular = 0.000  # Integral gain for angular velocity
        kd_angular = 0.065  # Derivative gain for angular velocity
        
        if self.target_coordinates[bot_name]:  # If the bot has a target
            target = self.target_coordinates[bot_name]  # Get the target position
            distance_to_target = sqrt((target[0] - current_position[0])**2 + (target[1] - current_position[1])**2)  # Calculate the distance to the target
            angle_to_target = atan2(target[1] - current_position[1], target[0] - current_position[0])  # Calculate the angle to the target
            angle_to_target = math.atan2(math.sin(angle_to_target), math.cos(angle_to_target))  # Normalize the angle to the target
            
            linear_error = distance_to_target  # Linear error is the distance to the target
            angular_error = angle_to_target - current_position[2]  # Angular error is the difference between the angle to the target and the current orientation
        
            # Calculate linear and angular velocities using PID controller
            if abs(angular_error) > error_threshold:
                derivative_error_y = angular_error - self.prev_error_y  # Calculate the derivative error for angular velocity
                self.integral_error_y += angular_error  # Update the integral error for angular velocity
                angular_velocity = kp_angular * angular_error + ki_angular * 0.1* self.integral_error_y + kd_angular * derivative_error_y / 0.1
                self.prev_error_y = angular_error  # Update the previous angular error
                derivative_error_x = linear_error - self.prev_error_x  # Calculate the derivative error for linear velocity
                self.integral_error_x += linear_error  # Update the integral error for linear velocity
                linear_velocity = kp_linear * linear_error + ki_linear * 0.1*self.integral_error_x + kd_linear * derivative_error_x / 0.1
                self.prev_error_x = linear_error  # Update the previous linear error
            else:
                derivative_error_x = linear_error - self.prev_error_x  # Calculate the derivative error for linear velocity
                self.integral_error_x += linear_error  # Update the integral error for linear velocity
                linear_velocity = kp_linear * linear_error + ki_linear * 0.1*self.integral_error_x + kd_linear * derivative_error_x / 0.1
                self.prev_error_x = linear_error  # Update the previous linear error
                self.integral_error_y = 0.0
            # Threshold clauses for stopping at the target
            if abs(angular_error) < error_threshold:  # If the bot is facing the target (small angular error)
                angular_velocity = 0.0  # Stop rotating
                self.integral_error_y = 0.0  # Reset the integral error for angular velocity
            if abs(linear_error) < error_threshold:  # If the bot is close to the target
                linear_velocity = 0.0  # Stop moving
                angular_velocity = 0.0  # Stop rotating
                self.integral_error_x = 0.0  # Reset the integral error for linear velocity
        
        else:  # If the bot does not have a target
            target1 = self.target_coordinates[f'artist{self.i - 1}']  # Get the target of the previous bot
            if self.i == (bots):
                target2 = self.target_coordinates[f'artist{1}']  # Wrap around to the first bot for bot 30
            else:
                target2 = self.target_coordinates[f'artist{self.i + 1}']  # Get the target of the next bot
            midpoint_x = (target1[0] + target2[0]) / 2  # Calculate the midpoint x-coordinate between the two targets
            midpoint_y = (target1[1] + target2[1]) / 2  # Calculate the midpoint y-coordinate between the two targets
            angle_to_midpoint = atan2(midpoint_y - current_position[1], midpoint_x - current_position[0])  # Calculate the angle to the midpoint
            distance_to_midpoint = sqrt((midpoint_x - current_position[0])**2 + (midpoint_y - current_position[1])**2)  # Calculate the distance to the midpoint

            angle_to_midpoint = math.atan2(math.sin(angle_to_midpoint), math.cos(angle_to_midpoint))  # Normalize the angle to the midpoint

            linear_error = distance_to_midpoint  # Linear error is the distance to the midpoint
            angular_error = angle_to_midpoint - current_position[2]  # Angular error is the difference between the angle to the midpoint and the current orientation

            # Calculate linear and angular velocities using PID controller
            if abs(angular_error) > error_threshold:
                derivative_error_y = angular_error - self.prev_error_y  # Calculate the derivative error for angular velocity
                self.integral_error_y += angular_error  # Update the integral error for angular velocity
                angular_velocity = kp_angular * angular_error + ki_angular * 0.1* self.integral_error_y + kd_angular * derivative_error_y / 0.1
                self.prev_error_y = angular_error  # Update the previous angular error
                derivative_error_x = linear_error - self.prev_error_x  # Calculate the derivative error for linear velocity
                self.integral_error_x += linear_error  # Update the integral error for linear velocity
                linear_velocity = kp_linear * linear_error + ki_linear * 0.1*self.integral_error_x + kd_linear * derivative_error_x / 0.1
                self.prev_error_x = linear_error  # Update the previous linear error
            else:
                derivative_error_x = linear_error - self.prev_error_x  # Calculate the derivative error for linear velocity
                self.integral_error_x += linear_error  # Update the integral error for linear velocity
                linear_velocity = kp_linear * linear_error + ki_linear * 0.1*self.integral_error_x + kd_linear * derivative_error_x / 0.1
                self.prev_error_x = linear_error  # Update the previous linear error
                self.integral_error_y = 0.0
            # Threshold clauses for stopping at the midpoint
            if abs(angular_error) < error_threshold:  # If the bot is facing the midpoint (small angular error)
                angular_velocity = 0.0  # Stop rotating
                self.integral_error_y = 0.0  # Reset the integral error for angular velocity
            if abs(linear_error) < error_threshold:  # If the bot is close to the midpoint
                linear_velocity = 0.0  # Stop moving
                angular_velocity = 0.0  # Stop rotating
                self.integral_error_x = 0.0  # Reset the integral error for linear velocity

        twist_msg = Twist()  # Create a new Twist message
        twist_msg.linear.x = linear_velocity  # Set the linear velocity in the twist message
        twist_msg.angular.z = angular_velocity  # Set the angular velocity in the twist message
        self.velocity_publisher.publish(twist_msg)  # Publish the twist message

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS Python client library with the provided arguments

    target_coordinates_list = {}  # Create an empty dictionary to store target coordinates
    for i in range(1, bots+1):  # Loop through bot numbers 1 to 30
        bot_name = f'artist{i}'  # Construct the bot name
        if (i % 2) != 0:  # Check if the bot number is odd
            target_coordinates_list[bot_name] = [1, 1]  # Set a default target position for odd-numbered bots
        else:
            target_coordinates_list[bot_name] = None  # Set the target to None for even-numbered bots

    artists = []  # Create an empty list to store ArtistBot instances
    executor = rclpy.executors.MultiThreadedExecutor()  # Create a MultiThreadedExecutor instance

    for bot_name in target_coordinates_list:  # Loop through the bot names in the target coordinates list
        artist = ArtistBot(bot_name, target_coordinates_list)  # Create an ArtistBot instance
        artists.append(artist)  # Add the ArtistBot instance to the list
        executor.add_node(artist)  # Add the ArtistBot node to the executor

    try:
        executor.spin()  # Start the executor and spin its nodes
    finally:
        for artist in artists:  # Loop through the ArtistBot instances
            artist.destroy_node()  # Destroy the ArtistBot node
        rclpy.shutdown()  # Shutdown the ROS Python client library

if __name__ == '__main__':
    main()  # Call the main function if this script is executed directly