

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>

#define WHEEL_RADIUS                     0.033


using std::cout;
using std::endl;

sensor_msgs::Imu imu;
sensor_msgs::JointState lastJointState;
sensor_msgs::JointState currJointState = lastJointState;
nav_msgs::Odometry odom;
ros::Publisher odom_pub;
geometry_msgs::TransformStamped odom_tf;
std::string odom_header_frame_id;
std::string odom_child_frame_id;
double orientation[4];
double odom_pose[3];
double odom_vel[3];

void init(ros::NodeHandle n);
bool calcOdometry();
void updateOdometry();

bool calcOdometry()
{
  
  double wheel_l, wheel_r;      // rotation value of void updateOdometry(void)wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  static double lastTime = ros::Time::now().toSec();
  

  double currTime;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  currTime = ros::Time::now().toSec();
  double diff_time = currTime - lastTime;
  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = (currJointState.position[0] - lastJointState.position[0]);
  wheel_r = (currJointState.position[1] - lastJointState.position[1]);

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;

  orientation[0] = imu.orientation.x;
  orientation[1] = imu.orientation.y;
  orientation[2] = imu.orientation.z;
  orientation[3] = imu.orientation.w;

  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_theta = theta;
  lastTime = currTime;
  return true;
}

void updateOdometry()
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

void updateTF()
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  currJointState = *msg;
  calcOdometry();
  updateOdometry();
  updateTF();
  odom.header.stamp = ros::Time::now();
  odom_pub.publish(odom);
  odom_tf.header.stamp = ros::Time::now();
  br.sendTransform(odom_tf);
  lastJointState = currJointState;

}
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu = *msg;
}

void init(ros::NodeHandle n)
{
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
  std::string get_tf_prefix;
  n.getParam("tf_prefix", get_tf_prefix);
  if (get_tf_prefix == "")
  {
    odom_header_frame_id = "odom";
    odom_child_frame_id = "base_footprint";  
  }
  else
  {
    odom_header_frame_id = get_tf_prefix + "/odom";
    odom_child_frame_id = get_tf_prefix + "/base_footprint";

  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sim_odom_pub");
  // std::this_thread::sleep_for(std::chrono::milliseconds(20000));
  ros::NodeHandle n;
  init(n);
  ros::Subscriber sub = n.subscribe("joint_states", 100 ,jointStateCallback);
  ros::Subscriber subImu = n.subscribe("imu", 100 , imuCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  cout << "init fin" << endl;
  ros::spin();
}