# ACSL_turtlebot3
Repo for ASCL_turtlebot3 code base

# Turtlebot
Original Author: [Walter Peregrim](https://github.com/walterperegrim)
Edited by: [Yi Han](https://github.com/hanyiabc), [Adwait Verulkar](https://github.com/adwaitverulkar), [Casper Gleich](https://github.com/CasperGleich)

# Install dependencies
The following dependencies need to be installed on the Remote PC in addition to ROS Melodic before the repository can be compiled without errors.
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-ros-msgs ros-melodic-cartographer-rviz ros-melodic-pid ros-melodic-qt-build
qt4-default
```

# To build for the first time
```
git clone https://github.com/hanyiabc/ACSL_turtlebot3.git
cd ACSL_turtlebot3
catkin_make
source devel/setup.bash
```
`setup.bash` file needs to be sourced for every terminal window, unless it is added to the `.bashrc` file.

TurtleBot3 has three models, Burger, Waffle, and Waffle Pi, 
so you have to set which model you want to use before you launch TurtleBot3. 
We added an additional model based on the Waffle Pi model. The new model supports customized differential drive controller and velocity effort PID control 

Type:

```gedit ~/.bashrc```

Then add the following line to the file:

```export TURTLEBOT3_MODEL=waffle_pi_effort_controller```

Save and close `.bashrc` file. Refresh environment variables by running the following command in the terminal:

```source ~/.bashrc```

# Running velocity effort PID control with Navigation Stack
The original TurtleBot3 code runs Mapping and Localization independently. This means, the environment first needs to be mapped and saved to a map file. Then this file is given as an input in the localization step where the robot can locate itself in the static map using a particle filter (Adaptive Monte Carlo Localization). 

However, our requirement is running SLAM algorithm on the robot, i.e. Simultaneous Localization and Mapping. Hence, AMCL has been removed and the transform obtained from any planning package (Cartographer, Hector, GMapping, etc.) is used to generate a dynamic map. The robot simulataneously localizes itself in this dynamic map.

## Running on Gazebo

1. Open a terminal and fire-up roscore.
```
roscore
```
2. In a new terminal launch the following file.
```
roslaunch turtlebot3_bringup turtlebot3_sim_nav_control.launch 
```
3. An ```rviz``` window will pop up and the bot will move based on the navigation goals provided.

## Running on TurtleBot3

1. **[Remote PC]** Run roscore.
```
roscore
```
2. **[TurtleBot PC]** ssh into TurtleBot SBC and launch the following file.
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
3. **[Remote PC]** In a new terminal, launch the following file.
```
roslaunch turtlebot3_bringup turtlebot3_physical_nav_control.launch 
```
4. An ```rviz``` window will pop up and the bot will move based on the navigation goals provided.


