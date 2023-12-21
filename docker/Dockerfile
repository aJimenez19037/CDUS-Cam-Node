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
RUN apt-get update && apt-get install -y \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran \
    && rm -rf /var/lib/apt/lists/

# Clone OpenCV and OpenCV contrib repositories
WORKDIR /opencv_build
RUN git clone --branch 4.7.0 --depth 1 https://github.com/opencv/opencv.git
RUN git clone --branch 4.7.0 --depth 1 https://github.com/opencv/opencv_contrib.git


# Build and install OpenCV with contrib modules
WORKDIR /home/opencv_build/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/opencv_sources/opencv_contrib/modules \
    -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D WITH_TBB=ON \
    -D WITH_V4L=ON \
    -D WITH_QT=OFF \
    -D WITH_OPENGL=ON \
    -D BUILD_opencv_java=OFF \
    -D BUILD_SHARED_LIBS=OFF \
    ..
RUN make -j$(nproc)
RUN make install


#----------------------------------------------------------------------------------INSTALL REALSENSE
WORKDIR /home/
COPY installLibrealsense/ .
WORKDIR /home/installLibrealsense
run ./installLibrealsense.sh
run ./buildLibrealsense.sh


COPY ../../catkin_ws/src/* /home/catkin_ws/src
WORKDIR /home/catkin_ws
RUN catkin_build