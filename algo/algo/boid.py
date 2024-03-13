#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class Boid:
    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity

class Flock:
    def __init__(self, num_boids):
        self.boids = [Boid(np.random.rand(2), np.random.rand(2)) for _ in range(num_boids)]

    def update_boids(self):
        for boid in self.boids:
            boid.position += boid.velocity
            boid.velocity += self.separation(boid) + self.alignment(boid) + self.cohesion(boid)

    def separation(self, boid):
        force = np.zeros(2)
        for other in self.boids:
            if other is not boid:
                difference = boid.position - other.position
                force += difference / np.linalg.norm(difference)
        return force

    def alignment(self, boid):
        average_velocity = np.mean([other.velocity for other in self.boids if other is not boid], axis=0)
        return average_velocity - boid.velocity

    def cohesion(self, boid):
        average_position = np.mean([other.position for other in self.boids if other is not boid], axis=0)
        return average_position - boid.position
class BoidsNode(Node):
    def __init__(self):
        super().__init__('boids_node')
        self.flock = Flock(num_boids=3)  # Initialize your flock here
        self.my_publishers = [self.create_publisher(Twist, f'/turtle{i+1}/cmd_vel', 10) for i in range(3)]

    def timer_callback(self):
        self.flock.update_boids()
        for i, boid in enumerate(self.flock.boids):
            msg = Twist()
            msg.linear.x = boid.velocity[0]  # Convert boid velocity to Twist message
            msg.angular.z = boid.velocity[1]
            self.my_publishers[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    boids_node = BoidsNode()
    timer_period = 0.1  # seconds
    boids_node.create_timer(timer_period, boids_node.timer_callback)
    rclpy.spin(boids_node)

if __name__ == '__main__':
    main()