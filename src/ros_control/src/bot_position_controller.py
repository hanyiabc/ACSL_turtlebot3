#!/usr/bin/env python3
import rospy
# rostopic pub /bot/joint_left_effort_controller/ std_msgs/Float64 -r 50 -- '-200.0'

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, JointState
from math import sin,cos,atan2,sqrt,fabs
from geometry_msgs.msg import Twist


def bot_joint_positions_publisher():
	rospy.init_node('joint_effort_node', anonymous=False)
	left_torque_pub = rospy.Publisher('/bot/joint_left_effort_controller/command', Float64, queue_size=100)
	right_torque_pub = rospy.Publisher('/bot/joint_right_effort_controller/command', Float64, queue_size=100)
	rate = rospy.Rate(50) #100 Hz

	while not rospy.is_shutdown():#for straight line
		left_torque_pub.publish(0.003)
		right_torque_pub.publish(0.003)


if __name__ == '__main__':
	try:
		bot_joint_positions_publisher()
	except rospy.ROSInterruptException:
		pass
