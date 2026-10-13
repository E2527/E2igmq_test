#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WS}/install/setup.bash" ]; then
  source "${WS}/install/setup.bash"
fi
exec "$@"
