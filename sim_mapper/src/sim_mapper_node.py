import rospy
from math import atan2, pi, copysign
from geometry_msgs.msg import Vector3, PoseStamped, Twist
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String, Float32MultiArray

NODE_NAME = "sim_mapper"
TOPIC_MOVE = "/robot1/cmd_vel"
TOPIC_LASER = "/robot1/laser"
TOPIC_POSE = "/robot1/pose"

# width of a tunnel in the simulation
PATH_MAX_WIDTH = 3
# high/low driving speed
MAX_SPEED = 2
MIN_SPEED = 1
# distance to wall when to switch to low driving speed
BRAKE_DISTANCE = 3
# max rotation speed
ROTATION_SPEED = 1
# resulting turn angle accuracy
TURN_ACCURACY = 0.05

distanceLeft = None
distanceRight = None
distanceForward = -1
def on_laser_data(data):
	global distanceLeft, distanceRight, distanceForward

	def distanceOrNone(angle):
		rad = angle * pi / 180
		index = int(abs(data.angle_min - rad) / data.angle_increment)
		distance = data.ranges[index]

		if distance < data.range_max:
			return distance
		else:
			return None

	distanceLeft = distanceOrNone(-90)
	distanceRight = distanceOrNone(90)
	distanceForward = distanceOrNone(0)

currentRotation = None
def on_pose(stampedData):
	global currentRotation
	data = stampedData.pose
	siny_cosp = 2 * data.orientation.w * data.orientation.z
	cosy_cosp = 1 - 2 * data.orientation.z * data.orientation.z
	currentRotation = atan2(siny_cosp, cosy_cosp)

def main():
	rospy.init_node(NODE_NAME)
	pub = rospy.Publisher(TOPIC_MOVE, Twist, queue_size=10)

	rospy.Subscriber(TOPIC_LASER, LaserScan, on_laser_data)
	rospy.Subscriber(TOPIC_POSE, PoseStamped, on_pose)

	rate = rospy.Rate(10)
	while currentRotation is None or distanceForward == -1:
		rate.sleep()

	targetRotation = currentRotation

	while not rospy.is_shutdown():
		if distanceForward is not None and distanceForward < PATH_MAX_WIDTH / 2:
			if distanceLeft is None or distanceLeft > PATH_MAX_WIDTH:
				targetRotation = targetRotation - pi/2
				rotSpeed = -ROTATION_SPEED
			elif distanceRight is None or distanceRight > PATH_MAX_WIDTH:
				targetRotation = targetRotation + pi/2
				rotSpeed = ROTATION_SPEED
			else:
				print("Reached a dead end :(")
				exit(1)

			if targetRotation > pi:
				targetRotation -= 2 * pi
			if targetRotation < -pi:
				targetRotation += 2 * pi

			lastDist = 2
			while lastDist > TURN_ACCURACY and not rospy.is_shutdown():
				lastDist = abs(currentRotation - targetRotation)
				if abs(lastDist) < pi / 8:
					rotSpeed = copysign(lastDist, rotSpeed)
				data = Twist(Vector3(0, 0, 0), Vector3(0, 0, rotSpeed))
				pub.publish(data)

				rate.sleep()
		else:
			if distanceForward is not None and distanceForward < BRAKE_DISTANCE:
				speedX = MIN_SPEED
			else:
				speedX = MAX_SPEED

		speedY = 0
		if distanceLeft is not None and distanceRight is not None \
				and distanceLeft < PATH_MAX_WIDTH and distanceRight < PATH_MAX_WIDTH:
			speedY = distanceRight - distanceLeft
			if abs(speedY) > 1:
				speedY = copysign(1, speedY)
			if abs(speedY) < 0.0003:
				speedY = 0


		data = Twist(Vector3(speedX, speedY, 0), Vector3(0, 0, 0))
		pub.publish(data)

		rate.sleep()

if __name__ == '__main__':
	main()
