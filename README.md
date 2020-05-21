# ROS Gremsy Gimbal Interface
A ROS interface to control Gremsy gimbals. Based on the [gSDK](https://github.com/gChuNguyen/gSDK_Linux) interface and the MavLink protocol.

Disclaimer: This software package is not officially developed by or related to Gremsy.

## Description
This package includes a ROS Node which warps the gSDK for the Gremsy Gimbals which are mainly used for physical image stabilization. 

The gimbal is connected via UART with a Linux host device running this node. 
Devices such as the Raspberry Pi feature a build-in UART interface others like most PCs or Laptops need a cheap USB Adapter. 
The used serial device, as well as many other gimbal specific parameters, can be configured in the `config.yaml` file.
The node publishes the gimbals encoder positions, imu measurements, and the camera mount orientation.

## Setup
Run the following commands to clone this repository and update all submodules (needed for the external gSDK repository).
```
git clone https://github.com/Flova/ros_gremsy
cd ros_gremsy
git submodule init
git submodule update
```

Now you need to install all dependencies using rosdep. To execute this command make sure that the correct catkin workspace is sourced and the repository you just cloned is (linked) inside the `src` directory.
```
rosdep install ros_gremsy
```

After installing the dependencies you should be able to build the package using:
```
catkin build ros_gremsy
```

## Launching
Type the following command to run the node. Make sure that the gimbal is connected properly, the Linux permissions regarding the serial interface are correct (this depends on your distro) and the config features the correct device and baudrate (default setting should be fine as far as I know).
```
roslaunch ros_gremsy gimbal.launch
```

## ROS Message API
The node publishes:
- `/gimbal/imu/data` with a [sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) message containing the raw gyro and accelerometer values. 
- `/gimbal/encoder` with a [geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html) message containing the encode values around the x (roll), y (pitch) and z (yaw) axis.
- `/gimbal/mount_orientation_global_yaw` with a [geometry_msgs/Quaternion](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html) message representing the camera mount orientation in the global frame. This measurement is imprecise in the yaw axis because of the gyro drift.
- `/gimbal/mount_orientation_local_yaw` with a [geometry_msgs/Quaternion](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html) message representing the camera mount orientation in the global frame except for the yaw axis which is provided relative to the gimbals mount on the vehicle or robot.

The node receives:
- `/gimbal/goals` expects a [geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html) message containing the desired angles for each axis. The frame for each axis (local or global), as well as the stabilization mode, can be configured in the `config.yaml` file.

## Further work
- Better dynamic reconfiguration
- Better handling of gimbal timestamps
