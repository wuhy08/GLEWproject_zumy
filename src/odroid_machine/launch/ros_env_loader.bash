#!/usr/bin/env bash

export MACHINE_TYPE=zumy
export COOP_SLAM_WORKSPACE=/home/$MACHINE_TYPE/coop_slam_workspace

source $COOP_SLAM_WORKSPACE/devel/setup.bash
export ROS_HOSTNAME=$HOSTNAME.local
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$COOP_SLAM_WORKSPACE
exec "$@"
