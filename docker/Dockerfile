FROM nvcr.io/nvidia/l4t-base:r32.7.1
#Issue with tzarguments when installing ROS

ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace
# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

#-----------------------------------------------------------------------------------ROS MELODIC INSTALL
#install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
    cmake \
    build-essential \
    curl \
    wget \ 
    ca-certificates \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/

ARG ROS_PKG=ros-base
ENV ROS_DISTRO=melodic

RUN curl -Ls https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# install ROS packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-melodic-${ROS_PKG} \
    ros-melodic-image-transport \
    ros-melodic-vision-msgs \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    python-catkin-tools \
    python-vcstool \
    && rm -rf /var/lib/apt/lists/*

# init/update rosdep
RUN rosdep init && \
    rosdep update

# setup entrypoint
WORKDIR /root/ros_workspace
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /opt/ros/'${ROS_DISTRO}'/setup.bash' >> /root/.bashrc 
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

#-----------------------------------------------------------------------------------EIGEN INSTALL
RUN apt-get update && apt-get install -y libeigen3-dev


#----------------------------------------------------------------------------------OPENCV AND CONTRIB INSTALL

ARG OPENCV_URL
ARG OPENCV_DEB

COPY opencv_install.sh /opt/opencv_install.sh
USER root
RUN chown root:root /opt/opencv_install.sh && chmod +x /opt/opencv_install.sh

# leave the installer script under /opt and patch it with the
# build args hardcoded in case it needs re-installed later
RUN cd /opt && \
    sed -i "2i OPENCV_URL=${OPENCV_URL}" opencv_install.sh && \
    sed -i "3i OPENCV_DEB=${OPENCV_DEB}" opencv_install.sh && \
    cat opencv_install.sh

RUN cd /opt && ./opencv_install.sh

#----------------------------------------------------------------------------------INSTALL REALSENSE
WORKDIR /home/
COPY installLibrealsense/ .
WORKDIR /home/installLibrealsense
run ./installLibrealsense.sh
run ./buildLibrealsense.sh


COPY ../../catkin_ws/src/* /home/catkin_ws/src
WORKDIR /home/catkin_ws
RUN catkin_build