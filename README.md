# gazebo_ros_tracked_vehicle_interface
This plugin makes the Gazebo SimpleTrackedVehicle Plugin available in ROS.

The plugin is mainly based on the gazebo_ros_diff_drive plugin (https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp).
Especially the odometry calculation is based on the plugin.

## Installation

The installation is a bit tricky because the build directory of the catkin workspace has to be added to the plugin path of gazebo and the plugin is not tested with all gazebo versions.
