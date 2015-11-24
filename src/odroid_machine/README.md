# Odroid_machine
A ROS node manages network communication for Zumy ROS project. It manages remote Zumy controlling launch, Zumy remote control environment setup, Zumy online connection detection, and Host-Zumy auto connection.

## Dependency
* python-nmap
* python-subprocess
* ros-indigo-zeroconf-avahi-suite

## Feature
* Zumy online_detector: publishes a ROS topic that shows all Zumies connected to host 
* Zumy auto_connect: allows host launches ROS in a Zumy as soon as it connects to host

## Limitation 
* ros_env path is not flexible because it is always set to: "/home/odroid/coop_slam_workspace/src/odroid_machine/launch/ros_env_loader.bash"
* ros_env_loader is not flexible, and its ROS_PACKAGE_PATH is always set to workspace "coop_slam_workspace"

## Known issues
* Auto_connect waits for a very long time for lost scan if lost Zumies are more than 2. The reason is that nmap spend a long to look for lost host. But this delay will not effect navigation control. 
* Sometimes fails to connect (need more tests)
