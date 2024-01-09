#!/bin/bash
set -e

# setup ros environment
if [ -e "./install/setup.bash" ]; then
  source  "./install/setup.bash"
else
  source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

colcon build
source "./install/setup.bash"

exec "$@"
