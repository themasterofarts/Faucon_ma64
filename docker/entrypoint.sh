#!/bin/bash
# Runtime entrypoint — sources ROS2 and the Faucon workspace before exec
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /opt/faucon/setup.bash ]; then
    source /opt/faucon/setup.bash
fi

exec "$@"
