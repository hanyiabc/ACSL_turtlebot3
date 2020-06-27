#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64
def left_callback(msg):
    print("left: " ,msg)

def right_callback(msg):
    print("right: " ,msg)

def state_callback(msg):
    print("state: " ,msg)

def cmd_callback(msg):


def listener():

    rospy.init_node('listener', anonymous=True)
    #rospy.Subscriber("bot/joint_right_position_controller/command", Float64, right_callback)
    #rospy.Subscriber("bot/joint_left_position_controller/command", Float64, left_callback)
    rospy.Subscriber("bot/joint_states", JointState, state_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
