# ACSL_turtlebot3
Repo for ASCL_turtlebot3 code base

# Turtlebot
Original Author: Walter Peregrim
Edited by: Yi Han

# Install dependencies
```sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers``` 

# To build for the first time

To compile:
```catkin_make```

TurtleBot3 has three models, Burger, Waffle, and Waffle Pi, 
so you have to set which model you want to use before you launch TurtleBot3. 
We added an additional model based on the Waffle Pi model. The new model supports differential drive controller and velocity  effort PID control 

Type:

```gedit ~/.bashrc```


Then add the following line to the file:

```export TURTLEBOT3_MODEL=waffle_pi_effort_controller```

Refresh environment variables:
```source ~/.bashrc```

## Instructions for running velocity effort PID control
In this configuration, SLAM is running without the initial map, the localization is changed from AMCL to the built-in localization from the SLAM algorithm. 
Run this command to launch everything including SLAM, navigation.

```roslaunch turtlebot3_navigation turtlebot3_navigation_no_map.launch```

Run this command to publish velocity to the right wheel

```rostopic pub /joint_right_velocity_controller/command std_msgs/Float64 "data: 2.0" ```

Similiarly, publish velocity to the right wheel
```rostopic pub /joint_left_velocity_controller/command std_msgs/Float64 "data: 2.0" ```

Check the information of the right wheel PID controller
```rostopic echo /joint_right_velocity_controller/state```

Similarly, for left wheel
```rostopic echo /joint_left_velocity_controller/state```

All the instructions below needs to be validated (because of the merge)

## Instructions for keyboard-controlled Gazebo Simulation
Open a new terminal

```roscore```

Open a new terminal and navigate to turtlebot

```source devel/setup.bash```

Launch the Gazebo environment:

```roslaunch turtlebot3_gazebo turtlebot3_house.launch```

To move the TurtleBot with your keyboard, use this command in another terminal tab:

```roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch```


## Instructions for running ros_node python scripts
Launch the Gazebo environment again using the same instructions as before.
Run the following commands for `subscriber.py` and `robot_control.py` respectively:

- ```rosrun ros_node subscriber.py```
- ```rosrun ros_node robot_control.py```

`subscriber.py` creates a ROS node and subscribes to a particular ROS topic. This
script is easily modifiable and gives a good idea of what type of information is
being passed between nodes.

`robot_control.py` instantiates a class that allows for the manipulation of the 
turtlebot's linear and angular velocity. This script concisely shows how the
dynamics of the turtlebot can be easily modified.

