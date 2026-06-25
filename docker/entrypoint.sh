#!/usr/bin/env bash
# Источает ROS и собранный workspace, затем выполняет переданную команду (CMD/command).
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
