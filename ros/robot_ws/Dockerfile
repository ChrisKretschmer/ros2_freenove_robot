FROM ros:humble

# install ros package
RUN apt-get update && apt-get install -y \
      libraspberrypi-bin libraspberrypi-dev \
      ros-${ROS_DISTRO}-rosbridge-suite \
      ros-${ROS_DISTRO}-librealsense2 \
      ros-${ROS_DISTRO}-realsense2-camera \
      ros-${ROS_DISTRO}-realsense2-camera-msgs \
      ros-${ROS_DISTRO}-v4l2-camera \
      ros-${ROS_DISTRO}-image-transport-plugins \
      python3-pip x11vnc git && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir ~/.vnc && x11vnc -storepasswd 12345 ~/.vnc/passwd
RUN pip3 install RPi.GPIO smbus

RUN mkdir -p /robot_ws/src
COPY ./src /robot_ws/src


RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /robot_ws && \
    rosdep install --from-paths src -y --ignore-src

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /robot_ws && \
    colcon build

# overwrite official entry point
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod a+x /ros_entrypoint.sh


# launch ros package
CMD ["ros2", "launch", "robot", "default.launch.py"]
