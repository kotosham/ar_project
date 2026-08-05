#!/usr/bin/env bash
# Source ROS and the built workspace, then execute the provided command.
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
