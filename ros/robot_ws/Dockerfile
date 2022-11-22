FROM ros:humble

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-rosbridge-suite \
      ros-${ROS_DISTRO}-v4l2-camera \
      python3-pip && \
    rm -rf /var/lib/apt/lists/*


RUN mkdir -p /robot_ws/src
COPY ./src /robot_ws/src

RUN pip3 install RPi.GPIO

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
CMD ["ros2", "launch", "robot", "default.launch.yaml"]
