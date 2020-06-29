#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <string.h>
#include <std_msgs/Float64.h>
#include <chrono>
#include <functional>
#include <ros/callback_queue.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#define WHEEL_RADIUS                     0.033

class Turtlebot3HWInterface : public hardware_interface::RobotHW
{
public:

    Turtlebot3HWInterface()
    {

        loadURDF(nh, "robot_description");
        // connect and register the joint state interface
        // hardware_interface::JointStateHandle state_handle_a("wheel_left_joint", &pos[0], &vel[0], &eff[0]);
        // jnt_state_interface.registerHandle(state_handle_a);

        // hardware_interface::JointStateHandle state_handle_b("wheel_right_joint", &pos[1], &vel[1], &eff[1]);
        // jnt_state_interface.registerHandle(state_handle_b);
        
        // registerInterface(&jnt_state_interface);

        // // connect and register the joint effort interface
        // hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("wheel_left_joint"), &cmd[0]);
        // jnt_eft_interface.registerHandle(pos_handle_a);

        // hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("wheel_right_joint"), &cmd[1]);
        // jnt_eft_interface.registerHandle(pos_handle_b);
        
        // registerInterface(&jnt_eft_interface);
        sub = n.subscribe("joint_states", 100 ,&Turtlebot3HWInterface::jointStateCallback, this);
        pubL = n.advertise<std_msgs::Float64>("/left_torque", 1000);
        pubR = n.advertise<std_msgs::Float64>("/right_torque", 1000);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        _eff[0] = msg->effort[0];
        _eff[1] = msg->effort[1];
        _pos[0] = msg->position[0] * WHEEL_RADIUS;
        _pos[1] = msg->position[1] * WHEEL_RADIUS;
        _vel[0] = msg->velocity[0] * WHEEL_RADIUS;
        _vel[1] = msg->velocity[1] * WHEEL_RADIUS;
    }
    void read(const ros::Duration &period)
    {
        
        vel[0] = _vel[0];
        vel[1] = _vel[1];
        eff[0] = _eff[0];
        eff[1] = _eff[1];
        pos[0] = _pos[0];
        pos[1] = _pos[1];
        

    }
    void write()
    {
        effortL.data = cmd[0];
        effortR.data = cmd[1];
        pubL.publish(effortL);
        pubR.publish(effortR);
    }
    ros::NodeHandle n;

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::EffortJointInterface jnt_eft_interface;

    ros::Subscriber sub;
    ros::Publisher pubL;
    ros::Publisher pubR;

    double _pos[2];
    double _vel[2];
    double _eff[2];

    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];

    std_msgs::Float64 effortL;
    std_msgs::Float64 effortR;

};

void controlLoop(Turtlebot3HWInterface &hw, controller_manager::ControllerManager &cm, std::chrono::system_clock::time_point &last_time)
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw.read(elapsed);
    cm.update(ros::Time::now(), elapsed);
    hw.write();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joint_hw_interface_node");
    Turtlebot3HWInterface hw;
    controller_manager::ControllerManager cm(&hw);

    double control_frequency;
    hw.n.param<double>("control_frequency", control_frequency, 100.0);

    ros::CallbackQueue my_robot_queue;
    ros::AsyncSpinner my_robot_spinner(1, &my_robot_queue);

    std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();
    ros::TimerOptions control_timer( ros::Duration(1 / control_frequency), std::bind(controlLoop, std::ref(hw), std::ref(cm), std::ref(last_time)), &my_robot_queue);
    ros::Timer control_loop = hw.n.createTimer(control_timer);
    my_robot_spinner.start();
    ros::spin();

    return 0;
}