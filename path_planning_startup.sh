#!/bin/bash 
source /opt/ros/foxy/setup.bash

BASEDIR=$(dirname "$0")
cd "$BASEDIR"
. install/setup.bash

ros2 launch launch/path_planning_launch_prod.py