# CDUS-Cam-Node
This repository containerizes L4T 32.7, OpenCV, ROS Melodic, Eigen, Realsense, and the PX4 Autopilot flight code. Additionally you can install the packages and run the code on your local copmputer. 

# Setting Up PX4 Sim 
```bash
#Clone PX4 Repo in ~/
$ git clone https://github.com/PX4/PX4-Autopilot.git
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


# Resources
PX4:

[SETTING UP PX4 GAZEBO SIM](https://docs.px4.io/v1.12/en/simulation/gazebo.html) 

[SETTING UP PX4 GAZEBO SIM WITH ROS AND MAVROS](https://docs.px4.io/v1.12/en/simulation/ros_interface.html)

[PX4 OFFBOARD EXAMPLE](https://docs.px4.io/main/en/ros/mavros_offboard_python.html)

OpenCV:

[OPENCV INSTALL ON UBUNTU 18.04 (NOT JETSON)](https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/)

[OPENCV 4.5 ON JETSON NANO](https://www.youtube.com/watch?v=P-EZr0zy53g)

Docker:

[GITHUB W/ JETSON CONTAINERS](https://github.com/dusty-nv/jetson-containers/blob/master/jetson_containers/l4t_version.py)

[UPGRADING TO NVIDIA CONTAINER](http://docs.nvidia.com/dgx/nvidia-container-runtime-upgrade/index.html)

