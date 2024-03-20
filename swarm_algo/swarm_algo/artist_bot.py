import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from artbotsim.msg import Pose
from artbotsim.msg import Target
import math
from math import atan2, sqrt
from math import pi as p
import re

class ArtistBot(Node):
    def __init__(self, bot_name, target_coordinates_list):
        super().__init__(bot_name)
        self.bot_name = bot_name
        self.target_coordinates = target_coordinates_list
        self.target = target_coordinates_list
        self.current = [0.0, 0.0, 0.0]
        self.i = 0
        self.letter=1
        self.target_subscriber = self.create_subscription(Target, 'target', self.target_callback, 10)
        self.pose_subscribers = self.create_subscription(Pose, f'{bot_name}/pose', self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, f'{bot_name}/cmd_vel', 10)
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.timer = self.create_timer(7.0, self.update_targets)
        
    def target_callback(self, msg):
        for i in range(1, 31):
            bot_name = f'artist{i}'
            if (i % 2) != 0:
                x = getattr(msg, f'target{i}_x')
                y = 720 - getattr(msg, f'target{i}_y')
                self.target[bot_name] = [x/64, y/48]
            else:
                self.target[bot_name] = None
		    
    def pose_callback(self, msg):
        bot_name = msg.src
        characters_to_remove = "[artist]"
        self.i = int(re.sub(characters_to_remove, "", bot_name))
        self.current[0] = msg.x
        self.current[1] = msg.y
        self.current[2] = msg.theta
        self.calculate_and_publish_velocity(bot_name)

    def update_targets(self):
        self.target_coordinates=self.target
        print(self.target_coordinates)
            
    def calculate_and_publish_velocity(self, bot_name):
        current_position = self.current
        if self.target_coordinates[bot_name]:
            target = self.target_coordinates[bot_name]
            distance_to_target = sqrt((target[0] - current_position[0])**2 + (target[1] - current_position[1])**2)
            angle_to_target = atan2(target[1] - current_position[1], target[0] - current_position[0])

            angle_to_target = math.atan2(math.sin(angle_to_target), math.cos(angle_to_target))

            kp_linear = 0.5
            ki_linear = 0.0001
            kd_linear = 0.2
            kp_angular = 4.0
            ki_angular = 0.0002
            kd_angular = 0.3

            linear_error = distance_to_target
            angular_error = angle_to_target - current_position[2]

            derivative_error_x = linear_error - self.prev_error_x
            derivative_error_y = angular_error - self.prev_error_y

            self.integral_error_x += linear_error
            self.integral_error_y += angular_error
            
           
            angular_velocity = kp_angular * angular_error + ki_angular * self.integral_error_y + kd_angular * derivative_error_y
            linear_velocity = kp_linear * linear_error + ki_linear * self.integral_error_x + kd_linear * derivative_error_x
            
            self.prev_error_x = linear_error
            self.prev_error_y = angular_error

            if abs(angle_to_target) < 0.01:
                angular_velocity = 0.0
            if distance_to_target < 0.01:
                linear_velocity = 0.0
                angular_velocity = 0.0
                self.integral_error_x = 0.0
                self.integral_error_y = 0.0

        else:
            target1 = self.target_coordinates[f'artist{self.i - 1}']
            if self.i == 30:
                target2 = self.target_coordinates[f'artist{1}']
            else:
                target2 = self.target_coordinates[f'artist{self.i + 1}']
            midpoint_x = (target1[0] + target2[0]) / 2
            midpoint_y = (target1[1] + target2[1]) / 2
            angle_to_midpoint = atan2(midpoint_y - current_position[1], midpoint_x - current_position[0])
            distance_to_midpoint = sqrt((midpoint_x - current_position[0])**2 + (midpoint_y - current_position[1])**2)

            angle_to_midpoint = math.atan2(math.sin(angle_to_midpoint), math.cos(angle_to_midpoint))

            kp_linear = 0.5
            ki_linear = 0.0001
            kd_linear = 0.2
            kp_angular = 4.0
            ki_angular = 0.0002
            kd_angular = 0.3

            linear_error = distance_to_midpoint
            angular_error = angle_to_midpoint - current_position[2]

            derivative_error_x = linear_error - self.prev_error_x
            derivative_error_y = angular_error - self.prev_error_y

            self.integral_error_x += linear_error
            self.integral_error_y += angular_error

       
            angular_velocity = kp_angular * angular_error + ki_angular * self.integral_error_y + kd_angular * derivative_error_y
            linear_velocity = kp_linear * linear_error + ki_linear * self.integral_error_x + kd_linear * derivative_error_x

            self.prev_error_x = linear_error
            self.prev_error_y = angular_error

            # Threshold clause
            if abs(angle_to_midpoint) < 0.01:
                angular_velocity = 0.0
            if distance_to_midpoint < 0.01:
                linear_velocity = 0.0
                angular_velocity = 0.0
                self.integral_error_x = 0.0
                self.integral_error_y = 0.0

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    target_coordinates_list = {}
    for i in range(1, 31):
        bot_name = f'artist{i}'
        if (i % 2) != 0:
            target_coordinates_list[bot_name] = [1,1]
        else:
            target_coordinates_list[bot_name] = None

    artists = []
    executor = rclpy.executors.MultiThreadedExecutor()
    for bot_name in target_coordinates_list:
        artist = ArtistBot(bot_name, target_coordinates_list)
        artists.append(artist)
        executor.add_node(artist)

    try:
        executor.spin()
    finally:
        for artist in artists:
            artist.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
