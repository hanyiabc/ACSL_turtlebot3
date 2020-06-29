#!/usr/bin/env python3
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, JointState
from math import sin,cos,atan2,sqrt,fabs
from geometry_msgs.msg import Twist

class Driver:
    def __init__(self):
        rospy.init_node('differential_driver')

        self._rate = rospy.get_param('~rate', 50)
        self._max_speed = rospy.get_param('~max_speed', 3)
        self._wheel_base = rospy.get_param('~wheel_base', 0.28)

        #self._left_speed_percent = 0
        #self._right_speed_percent = 0
        self._left_speed = 0
        self._right_speed = 0

        rospy.Subscriber('cmd_vel', Twist, self.velocity_received_callback)
        self.left_wheel_vel = rospy.Publisher('/bot/joint_left_velocity_controller/command', Float64, queue_size=100)
        self.right_wheel_vel  = rospy.Publisher('/bot/joint_right_velocity_controller/command', Float64, queue_size=100)

    def velocity_received_callback(self, message):
        linear = message.linear.x
        angular = message.angular.z

        self.left_speed = linear - angular*self._wheel_base/2
        self.right_speed = linear + angular*self._wheel_base/2

        #self._left_speed_percent = (100 * self.left_speed/self._max_speed)
        #self._right_speed_percent = (100 * self.right_speed/self._max_speed)
        print("left wheel velocity:",self.left_speed,"right wheel velocity:",self.right_speed)
        self.publish_velocity()

    def publish_velocity(self):
        self.left_wheel_vel.publish(self.left_speed)
        self.right_wheel_vel.publish(self.right_speed)

    def run(self):
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    driver = Driver()
    driver.run()
