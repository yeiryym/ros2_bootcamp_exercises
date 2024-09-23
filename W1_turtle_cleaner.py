#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleCircleCleaner(Node):

    def __init__(self):
        super().__init__('turtle_circle_cleaner')
        # Create a publisher to send velocity commands to the turtle
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Create a subscriber to get the turtle's current position (Pose)
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        # Create a timer that triggers the cleaning function every 0.1 second
        self.timer = self.create_timer(0.1, self.clean_circle)
       
        # Initialize twist message for controlling the turtle's motion
        self.cmd = Twist()
        # Initialize the turtle's current position (Pose)
        self.current_pose = Pose()

        # Parameters for controlling the movement
        self.linear_velocity = 0.2  # Initial linear velocity
        self.angular_velocity = 0.5  # Constant angular velocity for turning
        self.radius_increase_rate = 0.003  # Rate at which the radius (linear velocity) increases
        self.max_radius = 13.5  # Radius limit at which the turtle should stop
        self.stop_cleaning = False  # Flag to stop the cleaning process

    def update_pose(self, pose):
        # Update the current position of the turtle
        self.current_pose = pose

    def clean_circle(self):
        # If the turtle has already reached the boundary, stop it
        if self.stop_cleaning:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            return

        # Calculate the distance of the turtle from the center of the Turtlesim window
        distance_from_center = math.sqrt(self.current_pose.x**2 + self.current_pose.y**2)

        # Check if the turtle is within the maximum allowed radius
        if distance_from_center >= self.max_radius:
            self.get_logger().info(f"Turtle reached the boundary (radius {distance_from_center:.2f}). Stopping.")
            self.stop_cleaning = True
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            return

        # Increase linear velocity to create a spiral motion
        if self.linear_velocity < self.max_radius:
            self.linear_velocity += self.radius_increase_rate
        else:
            # Keep the radius constant when max_radius is reached
            self.linear_velocity = self.max_radius

        # Set linear velocity (move forward)
        self.cmd.linear.x = self.linear_velocity
        # Set constant angular velocity (for turning in a circle)
        self.cmd.angular.z = self.angular_velocity
       
        # Publish the velocity command to move the turtle
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    turtle_circle_cleaner = TurtleCircleCleaner()

    # Keep the node alive and processing the cleaning function
    rclpy.spin(turtle_circle_cleaner)

    # Shutdown when the cleaning is done
    turtle_circle_cleaner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()