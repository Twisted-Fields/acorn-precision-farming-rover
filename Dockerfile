# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:iron-ros-core-jammy

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-iron-ros-base=0.10.0-3* \
    && rm -rf /var/lib/apt/lists/*

# Everything above this line comes from:
# https://github.com/osrf/docker_images/blob/master/ros/iron/ubuntu/jammy/ros-base/Dockerfile

# Below this line, twistedfields additions.

RUN apt update && apt install -y libffi-dev \
                  python3-dev curl \
                  libkrb5-dev \
                  libzmq5 \
                  libblas-dev \
                  redis \
                  automake \
                  subversion \
                  libxml2-dev \
                  libxslt1-dev \
                  gfortran-arm-linux-gnueabi \
                  libjpeg8-dev \
                  python3-pip \
                  python3-scipy \
                  python3-numpy-dev \
                  tmux \
                  vim \
                  mercurial \
                  iw \
                  bash \
                  iputils-ping
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install wheel certifi==2020.06.20 pytest==6.2.5 coverage[toml]
COPY server/requirements.txt /install/server/
RUN python3 -m pip install -r /install/server/requirements.txt
COPY vehicle/requirements.txt /install/vehicle/
RUN python3 -m pip install -r /install/vehicle/requirements.txt

RUN python3 -m pip install adafruit-circuitpython-mcp230xx coloredlogs pyserial_asyncio pyubx2 pygame
RUN apt update && apt install -y libraspberrypi-bin; exit 0 # Only succeeds on raspberry pi but not needed otherwise.
RUN apt install -y iproute2
RUN python3 -m pip install utm crccheck
RUN python3 -m pip install gunicorn
RUN python3 -m pip install adafruit-circuitpython-lsm6ds pca9536-driver adafruit_extended_bus adafruit-circuitpython-lis3mdl
RUN python3 -m pip install adafruit-circuitpython-ads1x15
# RUN apt install -y software-properties-common
# RUN add-apt-repository ppa:deadsnakes/ppa && apt update && apt install -y python3.12-distutils python3.12
LABEL org.opencontainers.image.source=https://github.com/Twisted-Fields/acorn-precision-farming-rover
