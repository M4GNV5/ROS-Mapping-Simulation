import rospy
import numpy as np
from math import atan2, pi, sin, cos
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, PoseStamped, Twist
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import OccupancyGrid

NODE_NAME = "sim_cloud_creator"
TOPIC_GRID = "/robot1/grid"
TOPIC_LASER = "/robot1/laser"
TOPIC_POSE = "/robot1/pose"

occupancy_map = np.ones(2000000, dtype=np.int8).reshape(1000, 2000)
occupancy_map[:,:] = -1

SCALE_F = 20
RES_F = SCALE_F / 1000

def gp_com_message():
	""" Return a nav_msgs/OccupancyGrid representation of this map. """

	grid_msg = OccupancyGrid()
	grid_msg.header.stamp = rospy.Time.now()
	grid_msg.header.frame_id = "map"

	grid_msg.info.resolution = RES_F
	grid_msg.info.width = 2000
	grid_msg.info.height = 1000

	grid_msg.info.origin = Pose(Point(-5, -2.5, 0),
								Quaternion(0, 0, 0, 1))

	flat_grid = np.copy(occupancy_map.reshape((2000000,))) 
	flat_grid = flat_grid.astype(int)
	grid_msg.data = list(flat_grid)
	return grid_msg 

def update_occupanct_map(x1, y1, x2, y2):
	m = (y2 - y1) / (x2 - x1)
	t = y1 - m * x1

	x1 = int(x1)
	x2 = int(x2)

	for x in range(x1, x2):
		y = m * x + t

		xInt = int(x)
		yInt = int(y)

		#print(xInt,yInt)

		if xInt>=2000 or yInt>=1000 or xInt<0 or yInt < 0:
			continue

		occupancy_map[yInt, xInt] = 0

x_pos, y_pos, rotation = None,0,0
def on_laser_data(data):
	global x_pos, y_pos, rotation
	if x_pos is None:
		return

	for i, distance in enumerate(data.ranges):
		distance = distance * SCALE_F
		angle = data.angle_min + i*data.angle_increment
		angle_laser = rotation + angle
		if angle_laser>pi:
			angle_laser -= 2*pi
		if angle_laser<-pi:
			angle_laser += 2*pi

		dy = sin(angle_laser) * distance
		dx = cos(angle_laser) * distance

		x_point = x_pos + dx
		y_point = y_pos + dy

		update_occupanct_map(x_pos, y_pos, x_point, y_point)
		if distance<data.range_max:
			x_point = int(x_point)
			y_point = int(y_point)
			if x_point>=2000 or y_point>=1000 or x_point<0 or y_point < 0:
				continue
			occupancy_map[y_point, x_point] = 100 # oder 0

def on_pose(stampedData):
	global x_pos, y_pos, rotation

	data = stampedData.pose

	x_pos = data.position.x * SCALE_F
	y_pos = data.position.y * SCALE_F

	print(x_pos, y_pos)

	siny_cosp = 2 * data.orientation.w * data.orientation.z
	cosy_cosp = 1 - 2 * data.orientation.z * data.orientation.z
	rotation = atan2(siny_cosp, cosy_cosp)

def main():
	rospy.init_node(NODE_NAME)
	pub = rospy.Publisher(TOPIC_GRID, OccupancyGrid, queue_size=10)

	rospy.Subscriber(TOPIC_LASER, LaserScan, on_laser_data)
	rospy.Subscriber(TOPIC_POSE, PoseStamped, on_pose)

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		pub.publish(gp_com_message())
		rate.sleep()





if __name__ == '__main__':
	main()