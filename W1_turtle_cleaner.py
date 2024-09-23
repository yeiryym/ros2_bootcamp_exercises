#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleCircleCleaner(Node):

    def __init__(self):
        super().__init__('turtle_circle_cleaner')
        #
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        
        self.timer = self.create_timer(0.1, self.clean_circle)
       
        
        self.cmd = Twist()
        
        self.current_pose = Pose()

        
        self.linear_velocity = 0.2  # Initial linear velocity
        self.angular_velocity = 0.5  # angular velocity for turning
        self.radius_increase_rate = 0.003  # (linear velocity) increases
        self.max_radius = 13.5  
        self.stop_cleaning = False  

    def update_pose(self, pose):
        # Update the current position of the turtle
        self.current_pose = pose

    def clean_circle(self):
        
        if self.stop_cleaning:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            return

        
        distance_from_center = math.sqrt(self.current_pose.x**2 + self.current_pose.y**2)

        
        if distance_from_center >= self.max_radius:
            self.get_logger().info(f"Turtle reached the boundary (radius {distance_from_center:.2f}). Stopping.")
            self.stop_cleaning = True
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            return

       
        if self.linear_velocity < self.max_radius:
            self.linear_velocity += self.radius_increase_rate
        else:
            
            self.linear_velocity = self.max_radius

        # (move forward)
        self.cmd.linear.x = self.linear_velocity
        
        self.cmd.angular.z = self.angular_velocity
       
        
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    turtle_circle_cleaner = TurtleCircleCleaner()

    
    rclpy.spin(turtle_circle_cleaner)

   
    turtle_circle_cleaner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()