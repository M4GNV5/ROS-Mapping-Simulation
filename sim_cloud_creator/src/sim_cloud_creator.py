import rospy
import numpy as np
from math import atan2, pi, copysign
from geometry_msgs.msg import Vector3, Pose, Twist
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String, Float32MultiArray

NODE_NAME = "sim_cloud_creator"
TOPIC_MOVE = "/robot1/cmd_vel"
TOPIC_LASER = "/robot1/laser"
TOPIC_POSE = "/robot1/pose"

occupancy_map = np.array() # TODO...

def update_occupanct_map(x1, y1, x2, y2):
	pass # TODO

def on_laser_data(data):
	pass # TODO

def on_pose(data):
	pass # TODO

def on_pose(data):
	siny_cosp = 2 * data.orientation.w * data.orientation.z
	cosy_cosp = 1 - 2 * data.orientation.z * data.orientation.z
	rotation = atan2(siny_cosp, cosy_cosp)

def main():
	rospy.init_node(NODE_NAME)
	pub = rospy.Publisher(TOPIC_MOVE, Twist, queue_size=10)

	rospy.Subscriber(TOPIC_LASER, LaserScan, on_laser_data)
	rospy.Subscriber(TOPIC_POSE, Pose, on_pose)
	pass

if __name__ == '__main__':
	main()