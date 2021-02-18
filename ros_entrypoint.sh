#!/bin/bash
set -e

# setup ros environment
# shellcheck source=/opt/ros/$ROS_DISTRO/setup.bash
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_ws/devel/setup.bash"

socat tcp-listen:11411,reuseaddr,fork tcp-connect:"${MROS_ADDRESS}":11411 &
socat tcp-listen:11511,reuseaddr,fork tcp-connect:"${MROS_ADDRESS}":11511 &

exec "$@"
