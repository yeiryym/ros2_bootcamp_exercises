#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random

class RandomTurtleGoal(Node):
    def __init__(self):
        super().__init__("random_turtle_goal")
        
        
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        
        self.sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        # Store current position 
        self.current_pose = Pose()

        # random goal every 10 seconds
        self.timer = self.create_timer(10, self.set_random_goal)

        
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0

        # to know if a goal is set
        self.goal_set = False

        # to check and move towards the goal
        self.control_timer = self.create_timer(0.1, self.move_to_goal)

    def pose_callback(self, pose):
        
        self.current_pose = pose

    def set_random_goal(self):
        # Set random goal within the turtlesim boundaries
        self.goal_x = random.uniform(0.5, 10.5)
        self.goal_y = random.uniform(0.5, 10.5)
        self.goal_theta = random.uniform(-math.pi, math.pi)
        self.goal_set = True  # Mark that a goal is set
        

    def move_to_goal(self):
        if not self.goal_set:
            return  # if no goal is set

        tolerance = 0.1  
        vel_msg = Twist()

        # distance to the goal
        distance = math.sqrt((self.goal_x - self.current_pose.x)**2 + (self.goal_y - self.current_pose.y)**2)
        
        # angle to the goal
        angle_to_goal = math.atan2(self.goal_y - self.current_pose.y, self.goal_x - self.current_pose.x)
        angular_diff = angle_to_goal - self.current_pose.theta

        # Normalizing 
        angular_diff = math.atan2(math.sin(angular_diff), math.cos(angular_diff))

        # move forward
        if distance > tolerance:
            vel_msg.linear.x = 1.5 * distance  # Proportional linear velocity

        # rotate
        if abs(angular_diff) > 0.1:
            vel_msg.angular.z = 6.0 * angular_diff  # Proportional angular velocity
        else:
            vel_msg.angular.z = 0.0

        # publish velocity commands
        self.pub.publish(vel_msg)

        
        if self.reached_goal(tolerance):
            self.stop_turtle()
            self.goal_set = False  

    def reached_goal(self, tolerance):

        distance = math.sqrt((self.goal_x - self.current_pose.x)**2 + (self.goal_y - self.current_pose.y)**2)
        angle_to_goal = math.atan2(self.goal_y - self.current_pose.y, self.goal_x - self.current_pose.x)
        angular_diff = abs(angle_to_goal - self.current_pose.theta)
    
        return distance < tolerance and angular_diff < 0.1

    def stop_turtle(self):
    
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.pub.publish(vel_msg)
        self.get_logger().info("Goal reached. Turtle stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = RandomTurtleGoal()
    print("Publisher Node Runnning...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
