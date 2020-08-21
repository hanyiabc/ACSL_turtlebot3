#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sstream>
#include "../include/turtlegui/qnode.hpp"
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace turtlegui {

QNode::QNode(int argc, char** argv ) : init_argc(argc),init_argv(argv)
{	
}

void QNode::init()
{
	if(!ros::isStarted())
	{
		master_url = "http://192.168.1.158:11311/";
		host_url = "192.168.1.158";
		std::map<std::string,std::string> remappings;
		remappings["__master"] = master_url;
		remappings["__hostname"] = host_url;
		ros::init(init_argc, init_argv, "turtlegui");
		if ( ! ros::master::check() ) 
			std::cout << "Cannot find ROS Master:" << std::endl;

		ros::NodeHandle n;
		path_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);		//publisher for /initialpose
	}
}

QNode::~QNode() 
{}

void QNode::button_publish_path_pressed(std::vector<geometry_msgs::PoseWithCovarianceStamped> path)
{
	init();
	pub_msg = path;
	run();
}

void QNode::run() {

	ros::Rate loop_rate1(0.5);
	loop_rate1.sleep();
	ros::Rate loop_rate(10);

	int i = 0;
	while (ros::ok() && i < pub_msg.size())
	{
		
		path_publisher.publish(pub_msg[i]);
		ros::spinOnce();
		loop_rate.sleep();
		i++;
	}

	system("rostopic pub /path_ready std_msgs/Empty -1 &");
}

}  // namespace turtlegui
