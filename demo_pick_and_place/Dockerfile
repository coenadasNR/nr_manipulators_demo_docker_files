# Use the official ROS Humble desktop image as the base image
FROM osrf/ros:humble-desktop-full

ENV ROS_DISTRO=humble

COPY /src /root/ros2_ws/src/

# Set the working directory inside the container
WORKDIR /root/ros2_ws/src/

# Update apt-get and install dependencies for rosdep
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-librealsense2* \
    ros-humble-realsense2-* \
    python3-rosdep \
    python3-pip 


# Install opencv-contrib-python using pip3
RUN pip3 install opencv-contrib-python

RUN apt-get update && \
    apt install apparmor -y && \
    apt-get install -y \
    git \
    && rm -rf /etc/ros/rosdep/sources.list.d/20-default.list

# RUN git submodule update --init --recursive
# Initialize rosdep and update it
RUN rosdep init && rosdep update && rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

WORKDIR /root/ros2_ws/

RUN cd /root/ros2_ws \
    && . /opt/ros/humble/setup.sh \
    && colcon build

ENTRYPOINT [ "/root/ros2_ws/src/entrypoint.sh"]