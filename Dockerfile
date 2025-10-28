ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base-noble

SHELL ["/bin/bash", "-lc"]

RUN apt update
RUN apt install -y nlohmann-json3-dev libpcl-dev python3-pip \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    ros-${ROS_DISTRO}-rosbag2-storage \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins
RUN pip3 install rosbags --break-system-packages
RUN mkdir -p /test_ws/src
COPY mandeye_to_rosbag2 /test_ws/src/mandeye_to_rosbag2
COPY common /test_ws/src/common/
RUN cd /test_ws/ && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build


