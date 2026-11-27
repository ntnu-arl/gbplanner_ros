FROM mirror.gcr.io/library/ros@sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963

# Ubuntu 22.04 / ROS 2 Humble. The planner graph uses Boost Graph headers.
RUN apt-get update -qq \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       libboost-dev=1.74.0.3ubuntu7 libeigen3-dev=3.4.0-2ubuntu2 \
       ros-humble-tf2-ros ros-humble-visualization-msgs ros-humble-interactive-markers ros-humble-ament-cmake-gtest \
       ros-humble-trajectory-msgs ros-humble-robot-state-publisher ros-humble-ament-cmake-pytest python3-pytest \
       python3-colcon-common-extensions libgoogle-glog-dev libprotobuf-dev protobuf-compiler libbullet-dev libpcl-dev \
    && apt-get clean
WORKDIR /work/gbplanner_ros2/ros2
CMD ["bash"]
