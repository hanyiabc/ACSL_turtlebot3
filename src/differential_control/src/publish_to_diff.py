#!/usr/bin/env python3
import rospy
# rostopic pub /bot/joint_left_effort_controller/ std_msgs/Float64 -r 50 -- '-200.0'

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, JointState
from math import sin,cos,atan2,sqrt,fabs
from geometry_msgs.msg import Twist


def bot_joint_positions_publisher():
	rospy.init_node('cmd_vel_node', anonymous=False)
	cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

	rate = rospy.Rate(50) #100 Hz

	while not rospy.is_shutdown():#for straight line
		msg = Twist()
		msg.linear.x = 0.5
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = 0.2

		cmd_vel_pub.publish(msg)

if __name__ == '__main__':
	try:
		bot_joint_positions_publisher()
	except rospy.ROSInterruptException:
		pass
