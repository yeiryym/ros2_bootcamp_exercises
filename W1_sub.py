#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float64MultiArray


class StateSubscriber(Node): # initialize  node 
	def __init__(self):
		super().__init__("state_subscriber_node") # initializing node, "name of node"
		self.sub = self.create_subscription(Float64MultiArray, "random_dim_state", self.subscriber_callback, 10) # 
		self.pub = self.create_publisher(Float64MultiArray, "new_dim_state", 10) # publishes to topic new_dim_state

	def subscriber_callback(self, msg): # msg is variable where the data is being stored 
		state_7d = msg.data
		state_6d = state_7d[:6]
		new_msg = Float64MultiArray()  # Variable for the data type 
		new_msg.data = state_6d 
		self.pub.publish(new_msg) # Publish the data 
        



def main(args=None): 
	rclpy.init()
	state_subscriber_node = StateSubscriber()
	print("Speed Calculator Node Started")

	try: 
		rclpy.spin(state_subscriber_node)
	except KeyboardInterrupt: 
		print("Terminating Node...")
		state_subscriber_node.destroy_node()

if __name__ == '__main__':
	main()