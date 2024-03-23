# CDUS-Cam-Node
This repository containerizes L4T 32.7, OpenCV, ROS Melodic, Eigen, Realsense, and the PX4 Autopilot flight code. Additionally you can install the packages and run the code on your local copmputer. 

# Modifications needed for own setup


# ROS Melodic Installation

```bash
# ROS documentation found in ROS resources
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt install ros-melodic-ros-base # if on jetson otherwise feel free to install full version
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install ros-melodic-mavros # install mavros 
# create ros package
$ mkdir -p ~/catkin_ws/src
$ sudo apt install python-catkin-tools
$ cd ~/catkin_ws
$ catkin build
```

# Setting Up PX4 Sim 
```bash
#Clone PX4 Repo in ~/
$ git clone https://github.com/PX4/PX4-Autopilot.git --recursive
#Run Simulation 
$ cd /path/to/PX4-Autopilot
$ make px4_sitl gazebo
#Once "INFO  [commander] Ready for takeoff!" appears in your terminal
commander takeoff
# Watch the drone takeoff!!
```
# Setting UP PX4 Sim with ROS
Add the code below to the bottom of your .bashrc file 
```python
#Part of ROS melodic setup. 
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
#Add PX4-Autopilot package path to ROS so that ROS can access the package. This is necessary as the sim uses mavros_posix_sitl.launch, which uses other files located in the px4 package
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins
export PYTHONPATH=$PYTHONPATH:/usr/local/lib
```
In your terminal
```bash
$ source .bashrc
```
If you wish to modify the sim world, examples of this can be seen in the PX4 OFFBOARD EXAMPLE at the bottom of the README.

# Potential Errors
## Accessing DNN files
If your cam_node is having trouble opening the NNet files, open a new terminal, source your .bashrc file, and rebuild the container. 

# Resources
ROS:

[Installing ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

PX4:

[SETTING UP PX4 GAZEBO SIM](https://docs.px4.io/v1.12/en/simulation/gazebo.html) 

[SETTING UP PX4 GAZEBO SIM WITH ROS AND MAVROS](https://docs.px4.io/v1.12/en/simulation/ros_interface.html)

[PX4 OFFBOARD EXAMPLE](https://docs.px4.io/main/en/ros/mavros_offboard_python.html)

OpenCV:

[OPENCV INSTALL ON UBUNTU 18.04 (NON JETSON)](https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/)

[OPENCV 4.5 ON JETSON NANO](https://www.youtube.com/watch?v=P-EZr0zy53g)

RealSense:

[RealSense Install](https://jetsonhacks.com/2019/12/22/install-realsense-camera-in-5-minutes-jetson-nano/)

Docker:

[GITHUB W/ JETSON CONTAINERS](https://github.com/dusty-nv/jetson-containers/blob/master/jetson_containers/l4t_version.py)

[UPGRADING TO NVIDIA CONTAINER](http://docs.nvidia.com/dgx/nvidia-container-runtime-upgrade/index.html)

[PX4_CONTROLLER GUIDE](https://github.com/RuslanAgishev/px4_control/tree/master)

