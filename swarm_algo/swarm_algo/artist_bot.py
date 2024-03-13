import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from artbotsim.msg import Pose
from math import atan2, sqrt
import re

class ArtistBot(Node):
    def __init__(self, bot_name, target_coordinates_list):
        super().__init__(bot_name)
        self.bot_name = bot_name
        self.target_coordinates = target_coordinates_list
        self.current = [0.0, 0.0, 0.0]
        self.distances = {}
        self.pair = {name: [] for name in target_coordinates_list}
        self.i=0
        self.pose_subscribers = [self.create_subscription(Pose, f'{bot_name}/pose', self.pose_callback, 10)]
        self.velocity_publisher = self.create_publisher(Twist, f'{bot_name}/cmd_vel', 10)

    def pose_callback(self, msg):
        bot_name = self.bot_name
        characters_to_remove = "[artist]"
        self.i = int(re.sub(characters_to_remove, "", bot_name))
        self.current[0] = msg.x
        self.current[1] = msg.y
        self.current[2] = msg.theta
        # self.update_distances(bot_name)
        # if bot_name == self.bot_name:
        self.calculate_and_publish_velocity(bot_name)

    # def update_distances(self, bot_name):
    #     self.distances[bot_name] = {}
    #     for other_bot, other_pos in self.current.items():
    #         if other_bot != bot_name:
    #             distance = sqrt((self.current[bot_name][0] - other_pos[0])**2 + (self.current[bot_name][1] - other_pos[1])**2)
    #             self.distances[bot_name][other_bot] = distance

    # def find_nearest_available_pair(self, bot_name):
    #     sorted_distances = sorted(self.distances[bot_name].items(), key=lambda x: x[1])
    #     available_bots = [bot for bot, _ in sorted_distances if self.target_coordinates[bot]]
    #     self.assigned_pairs = set(pair for pairs in self.pair.values() for pair in pairs)

    #     for i in range(len(available_bots)):
    #         bot1 = available_bots[i]
    #         for j in range(i + 1, len(available_bots)):
    #             bot2 = available_bots[j]
    #             pair = (bot1, bot2)

    #             if pair not in self.assigned_pairs and bot1 not in self.pair.values() and bot2 not in self.pair.values():
    #                 self.pair[bot_name] = tuple(pair)
    #                 self.assigned_pairs.add(pair)
    #                 return

        # self.pair[bot_name] = [] 

    def calculate_and_publish_velocity(self, bot_name):
        current_position = {bot_name: self.current}
        current_angle = atan2(current_position[bot_name][1]-0.0, current_position[bot_name][0]-0.0)
        # print(current_position)
        if self.target_coordinates[bot_name]:
            target = self.target_coordinates[bot_name]
            distance_to_target = sqrt((target[0] - current_position[bot_name][0])**2 + (target[1] - current_position[bot_name][1])**2)
            angle_to_target = atan2(target[1] - current_position[bot_name][1], target[0] - current_position[bot_name][0])
            if distance_to_target > 0.1:
                linear_velocity=2.0
                angular_velocity = 0.5*(angle_to_midpoint-current_position[bot_name][2])
            else:
                linear_velocity = 0.0 
                angular_velocity = 0.0
        else:
            # if not self.pair[bot_name] or len(self.pair[bot_name]) < 2:
            #     self.find_nearest_available_pair(bot_name)

            # if self.pair[bot_name] and len(self.pair[bot_name]) == 2:
            # if (self.i-1)==0:
            #     lower='artist10'
            # else:
            #     lower=f'artist{self.i-1}'
            target1 = self.target_coordinates[f'artist{self.i-1}']
            target2 = self.target_coordinates[f'artist{self.i+1}']
            midpoint_x = (target1[0] + target2[0]) / 2
            midpoint_y = (target1[1] + target2[1]) / 2
            angle_to_midpoint = atan2(midpoint_y - current_position[bot_name][1], midpoint_x - current_position[bot_name][0])
            distance_to_midpoint = sqrt((midpoint_x - current_position[bot_name][0])**2 + (midpoint_y - current_position[bot_name][1])**2)
            if distance_to_midpoint > 0.1:
                linear_velocity = 1.0
                angular_velocity = 1*(angle_to_midpoint-current_position[bot_name][2])
            else:
                linear_velocity = 0.0 
                angular_velocity = 0.0
                
            # else:
            #     linear_velocity = 0.0
            #     angular_velocity = 0.0

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    target_coordinates_list = {}
    for i in range(1, 2):
        bot_name = f'artist{i}'
        if (i%2)!=0:
            target_coordinates_list[bot_name] = [i, i]
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