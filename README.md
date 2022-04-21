# gazebo_ros_tracked_vehicle_interface
This plugin makes the Gazebo SimpleTrackedVehicle Plugin available in ROS.

The plugin is mainly based on the gazebo_ros_diff_drive plugin (https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp).
Especially the odometry calculation is based on the plugin.

## Function Principle

Instead of reimplementing the plugin for simulating the track drive in gazebo like the standard gazebo_ros plugins (e.g. see DiffDrive), this plugin acts as an interface between the gazebo topics of SimpleTrackedVehicle and the ROS transport system (topics). By default the plugin subscribes to the ROS /cmd_vel topic (geometry_msgs::Twist) and forwards the command to the gazebo topic /cmd_vel_twist (gazebo transport msgs::Twist). It also publishes the odometry topic on /odom (nav_msgs::Odometry). The odomety source can either be the track speed of SimpleTrackedVehicle which is subject to slippage/inaccuracies or the absolute position of the model which can be used as ground truth for validation purposes.

<p align="center">
<img src="https://user-images.githubusercontent.com/46114370/164396182-c8fc5c27-5c1e-48b5-9ec2-f83936e082d2.png"/>
</p>

## Requirements

The plugin was developed and tested with ROS melodic - but other version should work fine.

It works with both Gazebo9 and Gazebo11 from the OSRF apt package list (packages.osrfoundation.org).

**It does not work with the binary package of Gazebo9 that comes with the default ROS melodic package list!** (see next chapter for installation instructions). The default version of Gazebo9 does not include the "msgs::Twist" message type.

## Installation

**1. (optional) Setup the OSRF package list and upgrade gazebo9**

Only if you have not yet added the package list to the the apt sources. Have a look at http://gazebosim.org/install which also describes the installation of gazebo binary packages. (page is offline at the time of writing - so commands are listed below)

Add the package list:
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Add the lists key:
```bash
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Rebuild the local index:
```bash
sudo apt-get update
```

Upgrade packages - gazebo9 should be included:
```bash
sudo apt-get upgrade
```

**2. (optional) Create a catkin workspace**

Create the standard catkin workspace. You can also create another workspace (e.g. ~/base_ws) and overlay it with your development workspace, so you don't have to recompile the plugin (see http://wiki.ros.org/catkin/Tutorials/workspace_overlaying)

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
```

**3. Download the source code**

```bash
git clone https://github.com/lprobsth/gazebo_ros_tracked_vehicle_interface.git
```

**4. Compile the package**

```bash
cd ~/catkin_ws # or /path/to/your/ws
source /opt/ros/melodic/setup.bash
catkin build # or catkin_make depending on the ros build tool you're using
```

**5. Setup the environment variables**

```bash
source ~/catkin_ws/devel/setup.bash # or /path/to/your/ws/devel/setub.bash
```

## Setup the plugin for your model

SDF example:
```xml
<sdf version='1.7'>
  <model name='your_model'>
    <...>
    
    <!-- setup of the SimpleTrackedVehicle plugin -->
    <plugin name='simple_tracked_vehicle' filename='libSimpleTrackedVehiclePlugin.so'>
      <body>chassis</body>
      <left_track>left_track</left_track>
      <right_track>right_track</right_track>
      <!-- <left_flipper>link_1</left_flipper> -->
      <!-- <right_flipper>link_1</right_flipper> -->
      <track_mu>2</track_mu>
      <track_mu2>2</track_mu2>
      <tracks_separation>0.571</tracks_separation>
    </plugin>
    
    <!-- setup of the interface -->
    <plugin name='tracked_vehicle_interface' filename='libgazebo_ros_tracked_vehicle_interface.so'>
      <commandROSTopic>/cmd_vel</commandROSTopic>
      <commandIGNTopic>~/cmd_vel_twist</commandIGNTopic>
      <!-- <odometryTopic>~odom</odometryTopic> -->     <!-- ROS topic for the odometry data - remap if you're using another odom source -->
      <!-- <trackSpeedTopic>~track_speed</trackSpeedTopic> -->  <!-- gazebo topic of SimpleTrackedVehicle -->
      
      <robotBaseFrame>base_link</robotBaseFrame>
      <!-- <odometryFrame>odom</odometryFrame> -->      <!-- tf frame of the odometry messages -->
      
      <tracks_separation>0.571</tracks_separation>
      <!--<publishOdomTF>true</publishOdomTF>  -->
      <!--<updateRate>100.0</updateRate>  -->
      
      <!-- <odometrySource>encoder</odometrySource> --> <!-- [encoder,world] - choose world for ground truth -->
    </plugin>
  </model>
</sdf>
```

## Troubleshooting

If the ROS topics of the plugin don't show up, gazebo probably could not find the plugin. During the compilation of the plugin an environment hook for catkin gets integrated into the setup file of the workspace that sets the environment variable with the path where gazebo should search for plugins. You can try and add the path `/path/to/your/ws/devel/lib/` to `GAZEBO_PLUGIN_PATH` (e.g. `export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/ubuntu/catkin_ws/devel/lib/`) manually.

## ToDo's

- [x] implement the environment hook that exports the GAZEBO_PLUGIN_PATH
