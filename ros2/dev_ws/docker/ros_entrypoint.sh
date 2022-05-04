#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/foxy/setup.bash"
source "/root/dev_ws/install/setup.bash"

exec "$@"
