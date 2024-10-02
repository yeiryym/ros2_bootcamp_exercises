#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import random
import math

class GoalMover(Node):
    def __init__(self):
        super().__init__('goal_mover')
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.movement_timer = self.create_timer(1, self.update_goal)
       
        # timer changes goal every 10 sec
        self.goal_change_timer = self.create_timer(10, self.change_goal)

        self.goal_x = random.uniform(-2, 2)
        self.goal_y = random.uniform(-2, 2)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.linear_speed_gain = 0.5 
        self.angular_speed_gain = 1.5 

        self.max_linear_speed = 0.8  
        self.max_angular_speed = 1.0 

    def odom_callback(self, msg):
        
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # yaw (rotation around z)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def update_goal(self):
        #  distance
        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

        # angular difference (how much we need to turn to face the goal)
        angle_diff = angle_to_goal - self.current_yaw

        # normalize
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        
        twist = Twist()

        
        if distance_to_goal > 0.1:  
            twist.linear.x = min(self.linear_speed_gain * distance_to_goal, self.max_linear_speed)
        else:
            twist.linear.x = 0.0 
       
        
        twist.angular.z = max(-self.max_angular_speed, min(self.angular_speed_gain * angle_diff, self.max_angular_speed))

        
        self.pub.publish(twist)

    def change_goal(self):
        
        self.goal_x = random.uniform(-2, 2)
        self.goal_y = random.uniform(-2, 2)
        self.get_logger().info(f'New goal set: x={self.goal_x}, y={self.goal_y}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalMover()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
   
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()