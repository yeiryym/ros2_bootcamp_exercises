#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float64MultiArray 

class StatePublisher(Node): 
	def __init__(self):
		super().__init__("state_publisher_node") # Node name 
		self.pub = self.create_publisher(Float64MultiArray, "random_dim_state", 10) # data type, name of topic
		self.timer = self.create_timer(0.5, self.publish_7_dim_state ) # sec, callback function
		self.counter = 0 

	def publish_7_dim_state (self):
		msg = Float64MultiArray()
		random_state = [random.uniform(-20, 20) for _ in range(7)]
		msg.data = random_state
		self.pub.publish(msg)
		self.counter += 1 
		

def main(args=None): 
	rclpy.init()
	my_pub = StatePublisher()
	print("Publisher Node Runnning...")

	try: 
		rclpy.spin(my_pub)
	except KeyboardInterrupt: 
		print("Terminating Node...")
		my_pub.destroy_node()

if __name__ == '__main__':
	main()