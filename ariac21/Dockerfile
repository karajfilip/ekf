FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04
LABEL maintainer filip.karaj@gmail.com

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS_DISTRO melodic
ENV TZ=Europe/Zagreb 

# Setup timezone (fix interactive package installation) 
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

# Install necessary packages for ROS and Gazebo 
RUN apt-get update &&  apt-get install -q -y \
    apt-utils \
    build-essential \
    bc \
    cmake \
    curl \
    git \
    gitk \
    geographiclib-doc \
    geographiclib-tools \
    imagemagick \
    lsb-release \
    libboost-dev \
    libgeographic-dev \
    libgeographic17 \
    node-geographiclib \
    sudo \
    nano \
    net-tools \
    tmux \ 
    wget 

# Add sources and keys for Gazebo installation 
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update

# Install gazebo packages 
RUN apt-get install -q -y \
    gazebo9 \
    libgazebo9-dev 

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update


# Install ROS
RUN apt-get install -y ros-${ROS_DISTRO}-desktop-full

# Install ROS python packages
RUN apt-get install -y \
    python-rosdep  \
    python-rosinstall \
    python-vcstool \
    python-rosinstall-generator  

# Install catkin tools 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y --no-install-recommends \
    python-catkin-tools \
    protobuf-compiler \
    libgoogle-glog-dev \
    libopencv-dev \
    python-pip

RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER developer 
WORKDIR /home/developer

RUN sudo rosdep init
RUN rosdep update


# Create a Catkin workspace
RUN mkdir -p catkin_ws/src
WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin init"

# Download Source Code
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/usnistgov/ARIAC.git \
        && git clone https://github.com/osrf/ariac-gazebo_ros_pkgs -b ariac-network-melodic

# Download universal_robot
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/ros-industrial/universal_robot.git

# Install GEAR Dependencies
WORKDIR /home/developer/catkin_ws
RUN rosdep install --from-path ./src --ignore-packages-from-source -y
RUN sudo apt install -y ros-melodic-ros-control* ros-melodic-control* ros-melodic-gazebo-ros-control*

# Build GEAR
WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin build"

CMD ["bash"] 