import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from artbotsim.msg import Pose
from math import atan2, sqrt, cos, sin

class ArtistBot(Node):
    def __init__(self, bot_name, target_coordinates_list):
        super().__init__(bot_name)
        self.target_coordinates = target_coordinates_list
        self.current = {name: [0, 0, 0, 0, 0] for name in target_coordinates_list}  # [x, y, theta, linear_velocity, angular_velocity]
        self.pose_subscriber = self.create_subscription(Pose, f'{bot_name}/pose', self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, f'{bot_name}/cmd_vel', 10)

    def pose_callback(self, msg):
        bot_name = self.get_name()
        self.current[bot_name][0] = msg.x
        self.current[bot_name][1] = msg.y
        self.current[bot_name][2] = msg.theta
        self.current[bot_name][3] = msg.linear_velocity
        self.current[bot_name][4] = msg.angular_velocity

        self.adjust_velocity_to_avoid_collision(bot_name)

    def adjust_velocity_to_avoid_collision(self, bot_name):
        bot_position = self.current[bot_name]
        other_bots = [name for name in self.current.keys() if name != bot_name]

        for other_bot in other_bots:
            other_position = self.current[other_bot]

            # Calculate distance between bots
            distance = sqrt((other_position[0] - bot_position[0]) ** 2 + (other_position[1] - bot_position[1]) ** 2)

            # Check if collision is imminent
            if distance < 0.5:  # Adjust the collision radius as needed
                # Calculate angle to avoid collision
                avoid_angle = atan2(bot_position[1] - other_position[1], bot_position[0] - other_position[0])

                # Calculate linear and angular velocity to avoid collision
                linear_velocity = 0.5  # Adjust the linear velocity as needed
                angular_velocity = 2 * (avoid_angle - bot_position[2])  # Adjust the angular velocity factor as needed

                # Publish the adjusted velocities
                twist_msg = Twist()
                twist_msg.linear.x = linear_velocity
                twist_msg.angular.z = angular_velocity
                self.velocity_publisher.publish(twist_msg)
                return

        # If no collision is detected, continue with normal behavior
        self.calculate_and_publish_velocity(bot_name)

    def calculate_and_publish_velocity(self, bot_name):
        target = self.target_coordinates[bot_name]
        current_position = self.current[bot_name]

        if not target:
            midpoint_x = (self.target_coordinates['artist1'][0] + self.target_coordinates['artist2'][0]) / 2
            midpoint_y = (self.target_coordinates['artist1'][1] + self.target_coordinates['artist2'][1]) / 2
            angle_to_midpoint = atan2(midpoint_y - current_position[1], midpoint_x - current_position[0])
            distance_to_midpoint = sqrt((midpoint_x - current_position[0]) ** 2 + (midpoint_y - current_position[1]) ** 2)
            if distance_to_midpoint <= 0.2:  # Adjust the distance threshold as needed
                linear_velocity = 0.0
                angular_velocity = 0.0
            else:
                linear_velocity = 1.0
                angular_velocity = 4 * (angle_to_midpoint - current_position[2])
        else:
            distance_to_target = sqrt((target[0] - current_position[0]) ** 2 + (target[1] - current_position[1]) ** 2)
            angle_to_target = atan2(target[1] - current_position[1], target[0] - current_position[0])
            if distance_to_target <= 0.2:  # Adjust the distance threshold as needed
                linear_velocity = 0.0
                angular_velocity = 0.0
            else:
                linear_velocity = 1.0
                angular_velocity = 4 * (angle_to_target - current_position[2])

        # Publish the calculated velocities
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    target_coordinates_list = {
        'artist1': [12.0, 12.0],
        'artist2': [3.0, 4.0],
        'artist3': None
    }
    artist1 = ArtistBot('artist1', target_coordinates_list)
    artist2 = ArtistBot('artist2', target_coordinates_list)
    artist3 = ArtistBot('artist3', target_coordinates_list)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(artist1)
    executor.add_node(artist2)
    executor.add_node(artist3)
    
    try:
        executor.spin()
    finally:
        artist1.destroy_node()
        artist2.destroy_node()
        artist3.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
