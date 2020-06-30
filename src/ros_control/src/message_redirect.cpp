

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <std_msgs/Float64.h>

#define WHEEL_RADIUS                     0.033



std_msgs::Float64 velL;
std_msgs::Float64 velR;
// double pos[2];
double vel[2];
// double eff[2];

ros::Publisher pubL;
ros::Publisher pubR;


void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // eff[0] = msg->effort[0];
    // eff[1] = msg->effort[1];
    // pos[0] = msg->position[0] * WHEEL_RADIUS;
    // pos[1] = msg->position[1] * WHEEL_RADIUS;
    vel[0] = msg->velocity[0] * WHEEL_RADIUS;
    vel[1] = msg->velocity[1] * WHEEL_RADIUS;

    velL.data = vel[0];
    velR.data = vel[1];
    pubL.publish(velL);
    pubR.publish(velR);
}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "joint_hw_interface_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joint_states", 100 ,jointStateCallback);
    pubL = n.advertise<std_msgs::Float64>("/left_vel_fb", 1000);
    pubR = n.advertise<std_msgs::Float64>("/right_vel_fb", 1000);
    ros::spin();

    return 0;
}