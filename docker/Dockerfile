FROM nvcr.io/nvidia/l4t-base:r32.7.1

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

#-----------------------------------------------------------------------------------EIGEN INSTALL
RUN apt-get update && apt-get install -y libeigen3-dev


# Setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    && rm -rf /var/lib/apt/lists/


# Initialize rosdep and update
RUN rosdep init && rosdep update

# Source the ROS environment in bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

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
RUN git clone --branch 4.8.0 --depth 1 https://github.com/opencv/opencv.git
RUN git clone --branch 4.8.0 --depth 1 https://github.com/opencv/opencv_contrib.git


# Build and install OpenCV with contrib modules
WORKDIR /opencv_build/opencv/build
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
COPY ../installLibrealsense/ 
WORKDIR installLibrealsense
run ./installLibrealsense.sh
run ./buildLibrealsense.sh




COPY ../catkin_ws/src/* /catkin_ws/src