#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/robot_ws/install/setup.bash" --

x11vnc -forever -usepw -create -rfbport 5900 &

exec "$@"
