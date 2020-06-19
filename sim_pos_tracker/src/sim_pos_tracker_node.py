import rospy, numpy as np
from math import sin, cos, atan2, pi
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3, PoseStamped
from icp import icp

NODE_NAME = "sim_pos_tracker"
TOPIC_LASER = "/robot1/laser"
TOPIC_POSE = "/robot1/tracked_pose"

last_laser_data = None
current_position = np.zeros(9).reshape(3, 3)
current_position[0][0] = 1
current_position[1][1] = 1
current_position[2][2] = 1

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
		T, _, _ = icp(B, last_laser_data, tolerance=0.000001)
		current_position = np.matmul(current_position, T)
		print(T)
		print(current_position)

	last_laser_data = B

def main():
	rospy.init_node(NODE_NAME)
	pub = rospy.Publisher(TOPIC_POSE, PoseStamped, queue_size=10)

	rospy.Subscriber(TOPIC_LASER, LaserScan, on_laser_data)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		theta = atan2(current_position[1][0], current_position[0][0])
		p = PoseStamped()
		p.header.frame_id = "pose"
		p.header.stamp = rospy.Time.now()
		p.pose.position.x = current_position[0][2]
		p.pose.position.y = current_position[1][2]
		p.pose.position.z = 0
		p.pose.orientation.w = cos(theta / 2.0)
		p.pose.orientation.x = 0
		p.pose.orientation.y = 0
		p.pose.orientation.z = sin(theta / 2.0)
		pub.publish(p)
		rate.sleep()

if __name__ == '__main__':
	main()