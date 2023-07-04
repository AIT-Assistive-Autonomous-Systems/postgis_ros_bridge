#!/bin/bash
set -e

./.github/setup.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
ament_${LINTER} ./
