import rospy, cv2, numpy as np, operator
from math import atan2, pi, sin, cos
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, PoseStamped, Twist
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import OccupancyGrid

NODE_NAME = "sim_cloud_creator"
TOPIC_GRID = "/robot1/grid"
TOPIC_LASER = "/robot1/laser"

# set TOPIC_POSE to "/robot1/pose" if you want a perfect but cheated result
# set TOPIC_POSE to "/robot/tracked_pose" if you want to use the pose tracked by sim_pos_tracker
TOPIC_POSE = "/robot1/tracked_pose"

# width/height of the map in pixel
WIDTH = 400
HEIGHT = 250

# width of the simulation in meter
WIDTH_METER = 18

SCALE_F = WIDTH / WIDTH_METER
RES_F = WIDTH_METER / WIDTH

occupancy_map = np.ones(WIDTH * HEIGHT, dtype=np.int8).reshape(HEIGHT, WIDTH)
occupancy_map[:,:] = -1

def gp_com_message():
	""" Return a nav_msgs/OccupancyGrid representation of this map. """

	grid_msg = OccupancyGrid()
	grid_msg.header.stamp = rospy.Time.now()
	grid_msg.header.frame_id = "pose"

	grid_msg.info.resolution = RES_F
	grid_msg.info.width = WIDTH
	grid_msg.info.height = HEIGHT

	grid_msg.info.origin = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

	flat_grid = np.copy(occupancy_map.reshape((WIDTH * HEIGHT,)))
	grid_msg.data = list(flat_grid)
	return grid_msg


def analyze_laser_data(x_curr, y_curr, rotation, data):
	for i, distance in enumerate(data.ranges):
		if distance >= data.range_max:
			continue

		distance = distance * SCALE_F
		angle = data.angle_min + i*data.angle_increment
		angle_laser = rotation + angle
		if angle_laser>pi:
			angle_laser -= 2*pi
		if angle_laser<-pi:
			angle_laser += 2*pi

		dy = sin(angle_laser) * distance
		dx = cos(angle_laser) * distance

		x_point = int(x_curr + dx)
		y_point = int(y_curr + dy)

		cv2.line(occupancy_map, (int(x_curr), int(y_curr)), (x_point, y_point), 1)

		if x_point >= WIDTH or y_point >= HEIGHT or x_point < 0 or y_point < 0:
			continue
		occupancy_map[y_point, x_point] = 100

laser_datasets = []
positions = []
def on_laser_data(data):
	global laser_datasets
	laser_datasets.append(data)


def on_pose(stampedData):
	global positions

	stamp = stampedData.header.stamp
	data = stampedData.pose

	x_pos = data.position.x * SCALE_F
	y_pos = data.position.y * SCALE_F

	siny_cosp = 2 * data.orientation.w * data.orientation.z
	cosy_cosp = 1 - 2 * data.orientation.z * data.orientation.z
	rotation = atan2(siny_cosp, cosy_cosp)

	positions.append((stamp, x_pos, y_pos, rotation))

def cmp_stamp(a, b):
	ds = a.secs - b.secs
	dns = a.nsecs - b.nsecs
	return ds * 1000 + dns / 1000000

def main():
	global positions, laser_datasets
	rospy.init_node(NODE_NAME)
	pub = rospy.Publisher(TOPIC_GRID, OccupancyGrid, queue_size=10)

	rospy.Subscriber(TOPIC_LASER, LaserScan, on_laser_data)
	rospy.Subscriber(TOPIC_POSE, PoseStamped, on_pose)

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		if len(positions) > 0 and len(laser_datasets) >= len(positions):

			for pos in positions:
				pos_stamp, x, y, rot = pos
				dataset = min(laser_datasets, key=lambda x: abs(cmp_stamp(x.header.stamp, pos_stamp)))

				if abs(cmp_stamp(dataset.header.stamp, pos_stamp)) > 1:
					continue

				laser_datasets.remove(dataset)
				positions.remove(pos)
				analyze_laser_data(x, y, rot, dataset)

			pub.publish(gp_com_message())
		rate.sleep()

if __name__ == '__main__':
	main()
