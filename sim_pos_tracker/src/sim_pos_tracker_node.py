import rospy, numpy as np
from math import sin, cos, atan2, pi
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3, PoseStamped
from icp import icp
from nav_msgs.msg import Path

NODE_NAME = "sim_pos_tracker"
TOPIC_LASER = "/robot1/laser"
TOPIC_POSE = "/robot1/tracked_pose"
TOPIC_PATH = "/robot1/path"

last_laser_data = None
current_position = np.zeros(9).reshape(3, 3)
current_position[0][0] = 1
current_position[1][1] = 1
current_position[2][2] = 1

current_position[0][2] = 2
current_position[1][2] = 2

pub = None
path = []

def on_laser_data(data):
	global last_laser_data, current_position

	points = []
	for i, distance in enumerate(data.ranges):
		angle = data.angle_min + i * data.angle_increment
		dy = sin(angle) * distance
		dx = cos(angle) * distance

		points.append((dx, dy))

	B = np.array(points)
	if last_laser_data is not None:
		T, _, _ = icp(B, last_laser_data, max_iterations=100,tolerance=0.000001)
		current_position = np.matmul(current_position, T)
		print(T)
		print(current_position)

		theta = atan2(current_position[1][0], current_position[0][0])
		p = PoseStamped()
		p.header.frame_id = "pose"
		p.header.stamp = data.header.stamp
		p.pose.position.x = current_position[0][2]
		p.pose.position.y = current_position[1][2]
		p.pose.position.z = 0
		p.pose.orientation.w = cos(theta / 2.0)
		p.pose.orientation.x = 0
		p.pose.orientation.y = 0
		p.pose.orientation.z = sin(theta / 2.0)
		path.append(p)
		pub.publish(p)

	last_laser_data = B

def main():
	global pub
	rospy.init_node(NODE_NAME)
	pub = rospy.Publisher(TOPIC_POSE, PoseStamped, queue_size=10)
	path_pub = rospy.Publisher(TOPIC_PATH, Path, queue_size=10)

	rospy.Subscriber(TOPIC_LASER, LaserScan, on_laser_data)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		p = Path()
		p.header.frame_id = "pose"
		p.header.stamp = rospy.Time.now()
		p.poses = path
		path_pub.publish(p)
		rate.sleep()

if __name__ == '__main__':
	main()
