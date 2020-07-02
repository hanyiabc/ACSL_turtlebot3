#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <math.h>
#include <bits/stdc++.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include <termios.h>


#define ELBOW 0 
#define GRIPPER 1
#define LEFT_FINGER 2 
#define RIGHT_FINGER 3
#define SHOULDER 4 
#define WAIST 5
#define WRIST_ANGLE 6
#define WRIST_ROTATE 7
#define PI 3.1428

using namespace std;

void dot_product(double a_matrix[5][5], double a_vector[5] , double out_vector[5]);
void sum_vector(double a_vec[5], double b_vec[5] , double c_vec[5] , double out_vec[5]);
void subs_vector(double a_vec[5], double b_vec[5], double out_vec[5]);
void cal_torque(double q[5], double q_dot[5], double torque[5]);

ofstream myfile;

ros::Publisher wx200_effort_pub;
ros::Subscriber sub;
std_msgs::Float64MultiArray msg;

char ch;
double shoulder_position = 0.0;
double elbow_position = 0.0;
double waist_position = 0.0;
double wrist_angle_position = 0.0;
double wrist_rotate_position = 0.0;

double q_ref[5] = {0};
double q_ref_dot[5] = {0};
double q_ref_ddot[5] = {0};

unsigned int publish_flag = 0;

double time_val = 0;

std::string datetime()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y %H-%M-%S",timeinfo);
    return std::string(buffer);
}

char getch()
{
  
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
	rv = -1;        
	//ROS_ERROR("select");
    else if(rv == 0)
	rv = 0;
        //ROS_INFO("no_key_pressed");
    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}


void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{

double calculated_torque[5] = {0};
double current_pos[5] = {0};
double current_vel[5] = {0};

  for(int i = 0; i< 8 ; i++)
  { 
	myfile << _js->name[i].c_str() << ": Position = " << _js->position[i] << " Effort = " << _js->effort[i] << " Velocity = " << _js->velocity[i] << "\n";
  }
 // myfile << "\n\n";


current_pos[0] = _js->position[WAIST];
current_pos[1] = _js->position[SHOULDER];
current_pos[2] = _js->position[ELBOW];
current_pos[3] = _js->position[WRIST_ANGLE];
current_pos[4] = _js->position[WRIST_ROTATE];

current_vel[0] = _js->velocity[WAIST];
current_vel[1] = _js->velocity[SHOULDER];
current_vel[2] = _js->velocity[ELBOW];
current_vel[3] = _js->velocity[WRIST_ANGLE];
current_vel[4] = _js->velocity[WRIST_ROTATE];

 cal_torque(current_pos, current_vel, calculated_torque);
  //wx200_effort_pub.publish(msg);
  //ROS_INFO("messaged_published");
  //ROS_INFO("Shoulder_position : %lf", _js->position[SHOULDER]);

  for(int i = 0; i< 5 ; i++)
  {
	msg.data[i] = calculated_torque[i];	
	myfile << "torque = " << calculated_torque[i] << "\n";
	myfile << "commanded position = " << q_ref[i] << "\n";
  }

	
  //if(publish_flag)
  wx200_effort_pub.publish(msg);
  myfile << "\n\n";
  time_val = time_val + 0.01;
  //loop_rate.sleep();
}



int main(int argc, char **argv)
{
	unsigned int t = 0;
		
	
	unsigned int loop = 0;
	ros::init(argc, argv, "wx200_effort");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	std::string fname;
	std::string path = "/home/burhan/interbotix_ws/Data_log/";
	fname = path + datetime() + ".txt";
	
	myfile.open (fname);
	sub = n.subscribe("/wx200/joint_states", 1, SetJointStates);
	wx200_effort_pub = n.advertise<std_msgs::Float64MultiArray>("/wx200/arm_controller/command",1);	
	msg.layout.dim.resize(5);
	msg.data.resize(5);
	msg.layout.dim[0].label = "waist";
	msg.layout.dim[0].size = 0;
	msg.layout.dim[0].stride = 0;
	msg.data[0] = 0;
	msg.layout.dim[1].label = "shoulder";
	msg.layout.dim[1].size = 0;
	msg.layout.dim[1].stride = 0;
	msg.data[1] = 0;
	msg.layout.dim[2].label = "elbow";
	msg.layout.dim[2].size = 0;
	msg.layout.dim[2].stride = 0;
	msg.data[2] = 0;
	msg.layout.dim[3].label = "wrist_angle";
	msg.layout.dim[3].size = 0;
	msg.layout.dim[3].stride = 0;
	msg.data[3] = 0;
	msg.layout.dim[4].label = "wrist_rotate";
	msg.layout.dim[4].size = 0;
	msg.layout.dim[4].stride = 0;
	msg.data[4] = 0;
	while(ros::ok())
	{
	
		//for(loop = 0; loop<100; loop++)
		//{
		publish_flag =0;
		t = 0;
		//q_ref[0] = sin(PI*time_val);
		//ROS_INFO("waist reference : %lf", q_ref[0]);
       		//q_ref_dot[0] = PI*cos(PI*time_val);
       		//q_ref_ddot[0] = -PI*PI*sin(PI*time_val);
		/*q_ref[1] = sin(0.2*PI*t);
       		q_ref_dot[1] = 0.2*PI * cos(0.2*PI*t);
       		q_ref_ddot[1] = -0.04*PI*PI*sin(0.2*PI*t);
		q_ref[2] = sin(0.2*PI*t);
       		q_ref_dot[2] = 0.2*PI * cos(0.2*PI*t);
       		q_ref_ddot[2] = -0.04*PI*PI*sin(0.2*PI*t);
		q_ref[3] = sin(0.2*PI*t);
       		q_ref_dot[3] = 0.2*PI * cos(0.2*PI*t);
       		q_ref_ddot[3] = -0.04*PI*PI*sin(0.2*PI*t);
		q_ref[4] = sin(0.2*PI*t);
       		q_ref_dot[4] = 0.2*PI * cos(0.2*PI*t);
       		q_ref_ddot[4] = -0.04*PI*PI*sin(0.2*PI*t);*/
		
		//ROS_INFO("time : %lf", time);
		
		ch = getch();
		if(ch == 'q')
		{
		    q_ref[0] = q_ref[0] + 0.010;
		    if(q_ref[0] >3.14)
			q_ref[0] = 3.14;
		    ROS_INFO("waist_position : %lf",q_ref[0]);		
		 }
		 else if(ch == 'a')
		 {
		    q_ref[0] = q_ref[0] - 0.010;
		    if(q_ref[0] < (-3.14))
			q_ref[0] = -3.14;
		    ROS_INFO("waist_position : %lf",q_ref[0]);	
		 }

		else if(ch == 'w')
		{
		    q_ref[1] = q_ref[1] + 0.010;
		    if(q_ref[1] > 1.97)
			q_ref[1] = 1.97;
		    ROS_INFO("Shoulder_position : %lf",q_ref[1]);		
		 }
		else if(ch == 's')
		 {
		    q_ref[1] = q_ref[1] - 0.010;
		    if(q_ref[1] < (-1.88))
			q_ref[1] = -1.88;
		    ROS_INFO("Shoulder_position : %lf",q_ref[1]);	
		 }

		else if(ch == 'e')
		{
		    q_ref[2] = q_ref[2] + 0.010;
		    if(q_ref[2] > 1.88)
			q_ref[2] = 1.88;
		    ROS_INFO("elbow_position : %lf",q_ref[2]);		
		 }
		 else if(ch == 'd')
		 {
		    q_ref[2] = q_ref[2] - 0.010;
		    if(q_ref[2] < (-1.62))
			q_ref[2] = -1.62;
		    ROS_INFO("elbow_position : %lf",q_ref[2]);	
		 }

		else if(ch == 'r')
		{
		    q_ref[3] = q_ref[3] + 0.010;
		    if(q_ref[3] > 1.75)
			q_ref[3] = 1.75;
		    ROS_INFO("wrist_angle_position : %lf",q_ref[3]);		
		 }
		 else if(ch == 'f')
		 {
		    q_ref[3] = q_ref[3] - 0.010;
		    if(q_ref[3] < (-2.15))
			q_ref[3] = -2.15;
		    ROS_INFO("wrist_angle_position : %lf",q_ref[3]);	
		 }
		else if(ch == 't')
		{
		    q_ref[4] = q_ref[4] + 0.010;
		    if(q_ref[4] > 3.14)
			q_ref[4] = 3.14;
		    ROS_INFO("wrist_rotate_position : %lf",q_ref[4]);		
		 }
		 else if(ch == 'g')
		 {
		    q_ref[4] = q_ref[4] - 0.010;
		    if(q_ref[4] < (-3.14))
			q_ref[4] = -3.14;
		    ROS_INFO("wrist_rotate_position : %lf",q_ref[4]);	
		 }
		//}/
	//publish_flag =0; 
		ros::spinOnce();
		//loop_rate.sleep();
	}

myfile.close();
//system("cd --"); 
return 0;	
}


void cal_torque(double q[5], double q_dot[5], double torque[5])
{
    double Kq[5] = {0};
    double C[5][5] = {0};
    double M[5][5] = {0};
    double q_ddot[5] = {0};
    double q_error[5] = {0};
    double q_dot_error[5] = {0};
    double out_vector_1[5] = {0};
    double out_vector_2[5] = {0};
    double KP[5][5] = {0};
    double KD[5][5] = {0};
    double m1 = 1.607196;
    double m2 = 0.715452;
    double m3 = 0.669306;
    double m4 = 0.416434;
    double m5 = 0.730841;

    double g = 9.81;

    double l1 = 0.11325;
    double l2 = 0.20616;
    double l3 = 0.2;
    double l4 = 0.065;
    double l5 = 0.10915;
    double lc1 = 0.047;
    double lc2 = 0.1353;
    double lc3 = 0.118;
    double lc4 = 0.0444;
    double lc5 = 0.0955;

    double I1 = 0.002175;
    double I2 = 0.00401991;
    double I3 = 0.00258893;
    double I4 = 0.00013472;
    double I5 = 0.00029738;

    unsigned int i = 0;
    unsigned int j = 0;


   KP[0][0] = 100;
   KP[1][1] = 2500;
   KP[2][2] = 2500;
   KP[3][3] = 700;
   KP[4][4] = 500;

   KD[0][0] = 1;
   KD[1][1] = 15;
   KD[2][2] = 15;
   KD[3][3] = 10;
   KD[4][4] = 5;
    /* equation for M */
    M[0][0] = I1 + I2 + I3 + I4 + I5 + (m4*pow(cos(q[0]),2)*pow((l3*cos(q[1] + q[2]) + l2*cos(q[1]) + lc4*cos(q[1] + q[2] + q[3])),2)) + (m5*pow(cos(q[0]),2)* pow((l3*cos(q[1] + q[2]) + l2*cos(q[1]) + lc5*cos(q[1] + q[2] + q[3])),2)) + (m4*pow(sin(q[0]),2)*pow((l3*cos(q[1] + q[2]) + l2*cos(q[1]) + lc4*cos(q[1] + q[2] + q[3])),2)) + (m5*pow(sin(q[0]),2)*pow((l3*cos(q[1] + q[2]) + l2*cos(q[1]) + lc5*cos(q[1] + q[2] + q[3])),2)) + (m3*pow(cos(q[0]),2)*pow((lc3*cos(q[1] + q[2]) + l2*cos(q[1])),2)) + (m3*pow(sin(q[0]),2)*pow((lc3*cos(q[1] + q[2]) + l2*cos(q[1])),2)) + (pow(lc2,2)*m2*pow(cos(q[0]),2)*pow(cos(q[1]),2)) + (pow(lc2,2)*m2*pow(cos(q[1]),2)*pow(sin(q[0]),2));
    M[0][4] = -I5*cos(q[1] + q[2] + q[3]);
    M[1][1] = I2 + I3 + I4 + I5 + pow(l2,2)*m3 + pow(l2,2)*m4 + pow(l2,2)*m5 + pow(l3,2)*m4 + pow(l3,2)*m5 + pow(lc2,2)*m2 + pow(lc3,2)*m3 + pow(lc4,2)*m4 + pow(lc5,2)*m5 + 2*l2*lc4*m4*cos(q[2] + q[3]) + 2*l2*lc5*m5*cos(q[2] + q[3]) + 2*l2*l3*m4*cos(q[2]) + 2*l2*l3*m5*cos(q[2]) + 2*l2*lc3*m3*cos(q[2]) + 2*l3*lc4*m4*cos(q[3]) + 2*l3*lc5*m5*cos(q[3]);
    M[1][2] = I3 + I4 + I5 + (pow(l3,2)*(m4 + m5)) + pow(lc3,2)*m3 + pow(lc4,2)*m4 + pow(lc5,2)*m5 + l2*lc4*m4*cos(q[2] + q[3]) + l2*lc5*m5*cos(q[2] + q[3]) + l2*l3*m4*cos(q[2]) + l2*l3*m5*cos(q[2]) + l2*lc3*m3*cos(q[2]) + 2*l3*lc4*m4*cos(q[3]) + 2*l3*lc5*m5*cos(q[3]);
    M[1][3] = I4 + I5 + pow(lc4,2)*m4 + pow(lc5,2)*m5 + (l2*cos(q[2] + q[3]) + l3 * cos(q[3])) * (m4*lc4 + m5 * lc5);
    M[2][1] = M[1][2];
    M[2][2] = I3 + I4 + I5 + (m4 + m5)*pow(l3,2) + m3*pow(lc3,2) + m4*pow(lc4,2) + m5*pow(lc5,2) + 2*l3*cos(q[3])*(m4*lc4 + m5*lc5);
    M[2][3] = I4 + I5 + m4*pow(lc4,2) + m5*pow(lc5,2) + l3*cos(q[3])*(m4*lc4 + m5*lc5);
    M[3][1] = M[1][3];
    M[3][2] = M[2][3];
    M[3][3] = I4 + I5 + m4*pow(lc4,2) + m5*pow(lc5,2);
    M[4][0] = M[0][4];
    M[4][4] = I5;

 /* equation for C */
    C[0][0] = (- (pow(l3,2)*m4*sin(2*q[1] + 2*q[2]))/2 - (pow(l3,2)*m5*sin(2*q[1] + 2*q[2]))/2 - (pow(lc3,2)*m3*sin(2*q[1] + 2*q[2]))/2 - (pow(l2,2)*m3*sin(2*q[1]))/2 - (pow(l2,2)*m4*sin(2*q[1]))/2 - (pow(l2,2)*m5*sin(2*q[1]))/2 - (pow(lc2,2)*m2*sin(2*q[1]))/2 - (pow(lc4,2)*m4*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - (pow(lc5,2)*m5*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - l2*lc4*m4*sin(2*q[1] + q[2] + q[3]) - l2*lc5*m5*sin(2*q[1] + q[2] + q[3]) - l3*lc4*m4*sin(2*q[1] + 2*q[2] + q[3]) - l3*lc5*m5*sin(2*q[1] + 2*q[2] + q[3]) - l2*l3*m4*sin(2*q[1] + q[2]) - l2*l3*m5*sin(2*q[1] + q[2]) - l2*lc3*m3*sin(2*q[1] + q[2]))*q_dot[1] + (- (pow(l3,2)*m4*sin(2*q[1] + 2*q[2]))/2 - (pow(l3,2)*m5*sin(2*q[1] + 2*q[2]))/2 - (pow(lc3,2)*m3*sin(2*q[1] + 2*q[2]))/2 - (pow(lc4,2)*m4*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - (pow(lc5,2)*m5*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - (l2*lc4*m4*sin(2*q[1] + q[2] + q[3]))/2 - (l2*lc5*m5*sin(2*q[1] + q[2] + q[3]))/2 - (l2*lc4*m4*sin(q[2] + q[3]))/2 - (l2*lc5*m5*sin(q[2] + q[3]))/2 - (l2*l3*m4*sin(q[2]))/2 - (l2*l3*m5*sin(q[2]))/2 - (l2*lc3*m3*sin(q[2]))/2 - l3*lc4*m4*sin(2*q[1] + 2*q[2] + q[3]) - l3*lc5*m5*sin(2*q[1] + 2*q[2] + q[3]) - (l2*l3*m4*sin(2*q[1] + q[2]))/2 - (l2*l3*m5*sin(2*q[1] + q[2]))/2 - (l2*lc3*m3*sin(2*q[1] + q[2]))/2)*q_dot[2] + (- (pow(lc4,2)*m4*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - (pow(lc5,2)*m5*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - (l2*lc4*m4*sin(2*q[1] + q[2] + q[3]))/2 - (l2*lc5*m5*sin(2*q[1] + q[2] + q[3]))/2 - (l2*lc4*m4*sin(q[2] + q[3]))/2 - (l2*lc5*m5*sin(q[2] + q[3]))/2 - (l3*lc4*m4*sin(q[3]))/2 - (l3*lc5*m5*sin(q[3]))/2 - (l3*lc4*m4*sin(2*q[1] + 2*q[2] + q[3]))/2 - (l3*lc5*m5*sin(2*q[1] + 2*q[2] + q[3]))/2)*q_dot[3];
    C[0][1] = (- (pow(l3,2)*m4*sin(2*q[1] + 2*q[2]))/2 - (pow(l3,2)*m5*sin(2*q[1] + 2*q[2]))/2 - (pow(lc3,2)*m3*sin(2*q[1] + 2*q[2]))/2 - (pow(l2,2)*m3*sin(2*q[1]))/2 - (pow(l2,2)*m4*sin(2*q[1]))/2 - (pow(l2,2)*m5*sin(2*q[1]))/2 - (pow(lc2,2)*m2*sin(2*q[1]))/2 - (pow(lc4,2)*m4*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - (pow(lc5,2)*m5*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - l2*lc4*m4*sin(2*q[1] + q[2] + q[3]) - l2*lc5*m5*sin(2*q[1] + q[2] + q[3]) - l3*lc4*m4*sin(2*q[1] + 2*q[2] + q[3]) - l3*lc5*m5*sin(2*q[1] + 2*q[2] + q[3]) - l2*l3*m4*sin(2*q[1] + q[2]) - l2*l3*m5*sin(2*q[1] + q[2]) - l2*lc3*m3*sin(2*q[1] + q[2]))*q_dot[0] + ((I5*sin(q[1] + q[2] + q[3]))/2)*q_dot[4];
    C[0][2] = (- (pow(l3,2)*m4*sin(2*q[1] + 2*q[2]))/2 - (pow(l3,2)*m5*sin(2*q[1] + 2*q[2]))/2 - (pow(lc3,2)*m3*sin(2*q[1] + 2*q[2]))/2 - (pow(lc4,2)*m4*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - (pow(lc5,2)*m5*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - (l2*lc4*m4*sin(2*q[1] + q[2] + q[3]))/2 - (l2*lc5*m5*sin(2*q[1] + q[2] + q[3]))/2 - (l2*lc4*m4*sin(q[2] + q[3]))/2 - (l2*lc5*m5*sin(q[2] + q[3]))/2 - (l2*l3*m4*sin(q[2]))/2 - (l2*l3*m5*sin(q[2]))/2 - (l2*lc3*m3*sin(q[2]))/2 - l3*lc4*m4*sin(2*q[1] + 2*q[2] + q[3]) - l3*lc5*m5*sin(2*q[1] + 2*q[2] + q[3]) - (l2*l3*m4*sin(2*q[1] + q[2]))/2 - (l2*l3*m5*sin(2*q[1] + q[2]))/2 - (l2*lc3*m3*sin(2*q[1] + q[2]))/2)*q_dot[0] + ((I5*sin(q[1] + q[2] + q[3]))/2)*q_dot[4];
    C[0][3] = (- (pow(lc4,2)*m4*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - (pow(lc5,2)*m5*sin(2*q[1] + 2*q[2] + 2*q[3]))/2 - (l2*lc4*m4*sin(2*q[1] + q[2] + q[3]))/2 - (l2*lc5*m5*sin(2*q[1] + q[2] + q[3]))/2 - (l2*lc4*m4*sin(q[2] + q[3]))/2 - (l2*lc5*m5*sin(q[2] + q[3]))/2 - (l3*lc4*m4*sin(q[3]))/2 - (l3*lc5*m5*sin(q[3]))/2 - (l3*lc4*m4*sin(2*q[1] + 2*q[2] + q[3]))/2 - (l3*lc5*m5*sin(2*q[1] + 2*q[2] + q[3]))/2)*q_dot[0] + ((I5*sin(q[1] + q[2] + q[3]))/2)*q_dot[4];
    C[0][4] = ((I5*sin(q[1] + q[2] + q[3]))/2)*q_dot[1] + ((I5*sin(q[1] + q[2] + q[3]))/2)*q_dot[2] + ((I5*sin(q[1] + q[2] + q[3]))/2)*q_dot[3];
    C[1][0] = -C[0][1];
    C[1][1] = (-l2*(l3*m4*sin(q[2]) + l3*m5*sin(q[2]) + lc3*m3*sin(q[2]) + lc4*m4*sin(q[2] + q[3]) + lc5*m5*sin(q[2] + q[3])))*q_dot[2] + (-(l2*sin(q[2] + q[3]) + l3*sin(q[3]))*(lc4*m4 + lc5*m5))*q_dot[3];
    C[1][2] = (-l2*(l3*m4*sin(q[2]) + l3*m5*sin(q[2]) + lc3*m3*sin(q[2]) + lc4*m4*sin(q[2] + q[3]) + lc5*m5*sin(q[2] + q[3])))*q_dot[1] + (-l2*(l3*m4*sin(q[2]) + l3*m5*sin(q[2]) + lc3*m3*sin(q[2]) + lc4*m4*sin(q[2] + q[3]) + lc5*m5*sin(q[2] + q[3])))*q_dot[2] + (-(l2*sin(q[2] + q[3]) + l3*sin(q[3]))*(lc4*m4 + lc5*m5))*q_dot[3];
    C[1][3] = (-(l2*sin(q[2] + q[3]) + l3*sin(q[3]))*(lc4*m4 + lc5*m5))*q_dot[1] + (-(l2*sin(q[2] + q[3]) + l3*sin(q[3]))*(lc4*m4 + lc5*m5))*q_dot[2] + (-(l2*sin(q[2] + q[3]) + l3*sin(q[3]))*(lc4*m4 + lc5*m5))*q_dot[3];
    C[1][4] = (-(I5*sin(q[1] + q[2] + q[3]))/2)*q_dot[0];
    C[2][0] = -C[0][2];
    C[2][1] = l2*(l3*m4*sin(q[2]) + l3*m5*sin(q[2]) + lc3*m3*sin(q[2]) + lc4*m4*sin(q[2] + q[3]) + lc5*m5*sin(q[2] + q[3]))*q_dot[1] + (-l3*sin(q[3])*(lc4*m4 + lc5*m5))*q_dot[3];
    C[2][2] = (-l3*sin(q[3])*(lc4*m4 + lc5*m5))*q_dot[3];
    C[2][3] = (-l3*sin(q[3])*(lc4*m4 + lc5*m5))*q_dot[1] + (-l3*sin(q[3])*(lc4*m4 + lc5*m5))*q_dot[2] + (-l3*sin(q[3])*(lc4*m4 + lc5*m5))*q_dot[3];
    C[2][4] = (-(I5*sin(q[1] + q[2] + q[3]))/2)*q_dot[0];
    C[3][0] = -C[0][3];
    C[3][1] = (l2*sin(q[2] + q[3]) + l3*sin(q[3]))*(lc4*m4 + lc5*m5)*q_dot[1] + l3*sin(q[3])*(lc4*m4 + lc5*m5)*q_dot[2];
    C[3][2] = l3*sin(q[3])*(lc4*m4 + lc5*m5)*q_dot[1] + l3*sin(q[3])*(lc4*m4 + lc5*m5)*q_dot[2];
    C[3][4] = (-(I5*sin(q[1] + q[2] + q[3]))/2)*q_dot[0];
    C[4][0] = C[0][4];
    C[4][1] = -C[1][4];
    C[4][2] = -C[2][4];
    C[4][3] = -C[3][4];

 /* equation for Kq */
    Kq[1] = g*(l2*m3*cos(q[1]) + l2*m4*cos(q[1]) + l2*m5*cos(q[1]) + lc2*m2*cos(q[1]) + lc4*m4*cos(q[1] + q[2] + q[3]) + lc5*m5*cos(q[1] + q[2] + q[3]) + l3*m4*cos(q[1] + q[2]) + l3*m5*cos(q[1] + q[2]) + lc3*m3*cos(q[1] + q[2]));
    Kq[2] = g*(lc4*m4*cos(q[1] + q[2] + q[3]) + lc5*m5*cos(q[1] + q[2] + q[3]) + l3*m4*cos(q[1] + q[2]) + l3*m5*cos(q[1] + q[2]) + lc3*m3*cos(q[1] + q[2]));
    Kq[3] = g*(cos(q[1]+q[2]+q[3]) *(lc4 * m4 + lc5 * m5) );


	//subs_vector(q_ref,q,q_error);
	//subs_vector(q_ref_dot,q_dot,q_dot_error);
	//dot_product(KP,q_error,out_vector_1);
	/*cout << "KP_0 = \n" << fixed << setprecision(4) << KP[0][0] << "\t" << KP[0][1] << "\t" << KP[0][2] << "\t" << KP[0][3] << "\t" << KP[0][4] << "\n";
	cout << "KP_1 = \n" << fixed << setprecision(4) << KP[1][0] << "\t" << KP[1][1] << "\t" << KP[1][2] << "\t" << KP[1][3] << "\t" << KP[1][4] << "\n";
	cout << "KP_2 = \n" << fixed << setprecision(4) << KP[2][0] << "\t" << KP[2][1] << "\t" << KP[2][2] << "\t" << KP[2][3] << "\t" << KP[2][4] << "\n";
	cout << "KP_3 = \n" << fixed << setprecision(4) << KP[3][0] << "\t" << KP[3][1] << "\t" << KP[3][2] << "\t" << KP[3][3] << "\t" << KP[3][4] << "\n";
	cout << "KP_4 = \n" << fixed << setprecision(4) << KP[4][0] << "\t" << KP[4][1] << "\t" << KP[4][2] << "\t" << KP[4][3] << "\t" << KP[4][4] << "\n\n";
	cout << "q_error = \n" << fixed << setprecision(4) << q_error[0] << "\t" << q_error[1] << "\t" << q_error[2] << "\t" << q_error[3] << "\t" << q_error[4] << "\n\n";*/
	//cout << "out_vector_1 = \n" << fixed << setprecision(4) << out_vector_1[0] << "\t" << out_vector_1[1] << "\t" << out_vector_1[2] << "\t" << out_vector_1[3] << "\t" << out_vector_1[4] << "\n\n";
	//dot_product(KD,q_dot_error,out_vector_2);
	//cout << "q_dot_error = \n" << fixed << setprecision(4) << q_dot_error[0] << "\t" << q_dot_error[1] << "\t" << q_dot_error[2] << "\t" << q_dot_error[3] << "\t" << q_dot_error[4] << "\n\n";
	//cout << "out_vector_2 = \n" << fixed << setprecision(4) << out_vector_2[0] << "\t" << out_vector_2[1] << "\t" << out_vector_2[2] << "\t" << out_vector_2[3] << "\t" << out_vector_2[4] << "\n\n";
	//cout << "q_ref_ddot = \n" << fixed << setprecision(4) << q_ref_ddot[0] << "\t" << q_ref_ddot[1] << "\t" << q_ref_ddot[2] << "\t" << q_ref_ddot[3] << "\t" << q_ref_ddot[4] << "\n\n";
	

	//sum_vector(out_vector_1,out_vector_2,q_ref_ddot,q_ddot);
	//cout << "q_ddot = \n" << fixed << setprecision(4) << q_ddot[0] << "\t" << q_ddot[1] << "\t" << q_ddot[2] << "\t" << q_ddot[3] << "\t" << q_ddot[4] << "\n\n";
	for(i = 0; i<5;i++)
	{
	    q_error[i] = q[i] - q_ref[i] ;
	    q_dot_error[i] = q_dot[i] - q_ref_dot[i];
	    q_ddot[i] = q_ref_ddot[i] - KP[i][i]*q_error[i] - KD[i][i]*q_dot_error[i]; 
	    //cout << fixed << setprecision(5) << q_ddot[i] << "\t";
	}
	//cout << "q_ddot = \n" << fixed << setprecision(4) << q_ddot[0] << "\t" << q_ddot[1] << "\t" << q_ddot[2] << "\t" << q_ddot[3] << "\t" << q_ddot[4] << "\n\n";

	//ROS_INFO("waist current : %lf", q[0]);
	//ROS_INFO("waist reference : %lf", q_ref[0]);
	//cout << "\n";
	//cout << fixed << setprecision(5) << "q = " << q_ref[0] << "\n" << "vel = " << q_ref_dot[0] << "\n" << "acc = " << q_ref_ddot[0] << "\n";

	for(i = 0; i <5; i++)
	{
		for(j=0;j<5;j++)
		{
			out_vector_1[i] = out_vector_1[i] + (M[i][j] * q_ddot[j]);  
			out_vector_2[i] = out_vector_2[i] + (C[i][j] * q_dot[j]); 			
		}
		torque[i] = out_vector_1[i] + out_vector_2[i] + Kq[i];
	}
	//dot_product(M,q_ddot,out_vector_1);
	//dot_product(C,q_dot,out_vector_2);
	//sum_vector(out_vector_1,out_vector_2,Kq,torque);
	//cout << "torque = \n" << fixed << setprecision(4) << torque[0] << "\t" << torque[1] << "\t" << torque[2] << "\t" << torque[3] << "\t" << torque[4] << "\n\n";
}



void dot_product(double a_matrix[5][5], double a_vector[5] , double out_vector[5])
{
    int i = 0;

    int j = 0;
    for(i = 0; i<5 ;i++)
    {
        for(j = 0;j<5;j++)
            out_vector[i] += a_matrix[i][j] * a_vector[j];
    }
}

void sum_vector(double a_vec[5], double b_vec[5] , double c_vec[5] , double out_vec[5])
{
    int i = 0;
    for(i =0 ; i<5 ; i++)
        out_vec[i] = a_vec[i]+b_vec[i]+c_vec[i];
}


void subs_vector(double a_vec[5], double b_vec[5], double out_vec[5])
{
    int i = 0;
    for(i =0 ; i<5 ; i++)
        out_vec[i] = a_vec[i] - b_vec[i];
}
