# ROS Gremsy Gimbal Interface
A ROS interface to control Gremsy V3 gimbals. Based on the [gSDK_V3_alpha](https://github.com/Gremsy/gSDK/tree/gSDK_V3_alpha) interface and the MavLink protocol.

Disclaimer: This software package is not officially developed by or related to Gremsy.

## Description
This package includes a ROS Node which warps the gSDK for the Gremsy V3 gimbals which are mainly used for physical image stabilization.

The gimbal is connected via UART with a Linux host device running this node.
Devices such as the Raspberry Pi feature a build-in UART interface others like most PCs or Laptops need a cheap USB Adapter.
The used serial device, as well as many other gimbal specific parameters, can be configured in the `config.yaml` file.
The node publishes the gimbals encoder positions.

## Setup
Run the following commands to clone this repository and update all submodules (needed for the external gSDK repository).
```
git clone --recurse-submodules -b ros_gremsy_V3 https://github.com/Flova/ros_gremsy.git
cd ros_gremsy
```

Now you need to install all dependencies using rosdep. To execute this command make sure that the correct catkin workspace is sourced and the repository you just cloned is (linked) inside the `src` directory.
```
rosdep install --from-paths . --ignore-src -r -y
```

After installing the dependencies you should be able to build the package using:
```
catkin build
```

## Launching
Type the following command to run the node. Make sure that the gimbal is connected properly, the Linux permissions regarding the serial interface are correct (this depends on your distro) and the config features the correct device and baudrate (default setting should be fine as far as I know).
```
roslaunch gremsy_bringup gimbal.launch
```

## ROS Message API
The node publishes:
- `/ros_gremsy/encoder` with a [geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html) message containing the encode values around the x (roll), y (pitch) and z (yaw) axis.

The node receives:
- `/ros_gremsy/goals` expects a [geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html) message containing the desired angles for each axis. The frame for each axis (local or global), as well as the stabilization mode, can be configured in the `config.yaml` file.

## Further work
- Better dynamic reconfiguration
- Better handling of gimbal timestamps
- Publication of imu data and camera orientation
