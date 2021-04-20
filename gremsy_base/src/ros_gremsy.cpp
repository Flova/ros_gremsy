#include <gremsy_base/ros_gremsy.h>

GimbalNode::GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    // Initialize dynamic-reconfigure
    dynamic_reconfigure::Server<gremsy_base::ROSGremsyConfig> server(pnh);
    dynamic_reconfigure::Server<gremsy_base::ROSGremsyConfig>::CallbackType f;
    f = boost::bind(&GimbalNode::reconfigureCallback, this, _1, _2);
    server.setCallback(f);

    // Advertive Publishers
    imu_pub = nh.advertise<sensor_msgs::Imu>("/ros_gremsy/imu/data", 10);
    encoder_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/ros_gremsy/encoder", 1000);
    mount_orientation_incl_global_yaw = nh.advertise<geometry_msgs::Quaternion>("/ros_gremsy/mount_orientation_global_yaw", 10);
    mount_orientation_incl_local_yaw = nh.advertise<geometry_msgs::Quaternion>("/ros_gremsy/mount_orientation_local_yaw", 10);


    // Register Subscribers
    gimbal_goal_sub = nh.subscribe("/ros_gremsy/goals", 1, &GimbalNode::setGoalsCallback, this);

    // Define SDK objects
    serial_port_ = new Serial_Port(config_.device.c_str(), config_.baudrate);
    gimbal_interface_ = new Gimbal_Interface(serial_port_);

    // Start ther serial interface and the gimbal SDK
    serial_port_->start();
    gimbal_interface_->start();

    ///////////////////
    // Config Gimbal //
    ///////////////////

    // Check if gimbal is on
    if(gimbal_interface_->get_gimbal_status().mode == GIMBAL_STATE_OFF)
    {
        // Turn on gimbal
        ROS_INFO("TURN_ON!\n");
        gimbal_interface_->set_gimbal_motor_mode(TURN_ON);
    }

    // Wait until the gimbal is on
    while (gimbal_interface_->get_gimbal_status().mode < GIMBAL_STATE_ON)
    {
        ros::Duration(0.2).sleep();
    }

    // Set gimbal control modes

    gimbal_interface_->set_gimbal_mode(convertIntGimbalMode(config_.gimbal_mode));

    // Set modes for each axis

    control_gimbal_axis_mode_t tilt_axis_mode, roll_axis_mode, pan_axis_mode;

    tilt_axis_mode.input_mode = convertIntToAxisInputMode(config_.tilt_axis_input_mode);
    tilt_axis_mode.stabilize = config_.tilt_axis_stabilize;

    roll_axis_mode.input_mode = convertIntToAxisInputMode(config_.roll_axis_input_mode);
    roll_axis_mode.stabilize = config_.roll_axis_stabilize;

    pan_axis_mode.input_mode = convertIntToAxisInputMode(config_.pan_axis_input_mode);
    pan_axis_mode.stabilize = config_.pan_axis_stabilize;

    gimbal_interface_->set_gimbal_axes_mode(tilt_axis_mode, roll_axis_mode, pan_axis_mode);

    ros::Timer timer = nh.createTimer(
        ros::Duration(1/config_.state_poll_rate),
        &GimbalNode::gimbalStateTimerCallback, this);

    ros::spin();
}

void GimbalNode::gimbalStateTimerCallback(const ros::TimerEvent& event)
{
    // Publish Gimbal IMU
    mavlink_raw_imu_t imu_mav = gimbal_interface_->get_gimbal_raw_imu();
    imu_mav.time_usec = gimbal_interface_->get_gimbal_time_stamps().raw_imu; // TODO implement rostime
    sensor_msgs::Imu imu_ros_mag = convertImuMavlinkMessageToROSMessage(imu_mav);
    imu_pub.publish(imu_ros_mag);

    // Publish Gimbal Encoder Values
    mavlink_mount_status_t mount_status = gimbal_interface_->get_gimbal_mount_status();
    geometry_msgs::Vector3Stamped encoder_ros_msg;
    encoder_ros_msg.header.stamp = ros::Time::now();
    encoder_ros_msg.vector.x = ((float) mount_status.pointing_b) * DEG_TO_RAD;
    encoder_ros_msg.vector.y = ((float) mount_status.pointing_a) * DEG_TO_RAD;
    encoder_ros_msg.vector.z = ((float) mount_status.pointing_c) * DEG_TO_RAD;
    // encoder_ros_msg.header TODO time stamps

    encoder_pub.publish(encoder_ros_msg);

    // Get Mount Orientation
    mavlink_mount_orientation_t mount_orientation = gimbal_interface_->get_gimbal_mount_orientation();

    // Publish Camera Mount Orientation in global frame (drifting)
    mount_orientation_incl_global_yaw.publish(
        tf2::toMsg(
            convertXZYtoQuaternion(
                mount_orientation.roll,
                mount_orientation.pitch,
                mount_orientation.yaw_absolute)));

    // Publish Camera Mount Orientation in local frame (yaw relative to vehicle)
    mount_orientation_incl_global_yaw.publish(
        tf2::toMsg(
            convertXZYtoQuaternion(
                mount_orientation.roll,
                mount_orientation.pitch,
                mount_orientation.yaw)));
}

Eigen::Quaterniond GimbalNode::convertXZYtoQuaternion(double roll, double pitch, double yaw)
{
    Eigen::Quaterniond quat_abs(
                  Eigen::AngleAxisd(-DEG_TO_RAD * pitch, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(-DEG_TO_RAD * roll, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(DEG_TO_RAD * yaw, Eigen::Vector3d::UnitZ())));
    return quat_abs;
}

void GimbalNode::setGoalsCallback(geometry_msgs::Vector3Stamped message)
{
    gimbal_interface_->set_gimbal_move(
        RAD_TO_DEG * message.vector.y,
        RAD_TO_DEG * message.vector.x,
        RAD_TO_DEG * message.vector.z);
}

sensor_msgs::Imu GimbalNode::convertImuMavlinkMessageToROSMessage(mavlink_raw_imu_t message)
{
    sensor_msgs::Imu imu_message;

    // Set accelaration data
    imu_message.linear_acceleration.x = message.xacc;
    imu_message.linear_acceleration.y = message.yacc;
    imu_message.linear_acceleration.z = message.zacc;

    // Set gyro data
    imu_message.angular_velocity.x = message.xgyro;
    imu_message.angular_velocity.y = message.ygyro;
    imu_message.angular_velocity.z = message.zgyro;

    return imu_message;
}

control_gimbal_mode_t GimbalNode::convertIntGimbalMode(int mode)
{   // Allows int access to the control_gimbal_mode_t struct
    switch(mode) {
        case 0 : return GIMBAL_OFF;
        case 1 : return LOCK_MODE;
        case 2 : return FOLLOW_MODE;
        default:
            ROS_ERROR_ONCE("Undefined gimbal mode used. Check the config file.");
            return GIMBAL_OFF;
    }
}

control_gimbal_axis_input_mode_t GimbalNode::convertIntToAxisInputMode(int mode)
{   // Allows int access to the control_gimbal_axis_input_mode_t struct
    switch(mode) {
        case 0 : return CTRL_ANGLE_BODY_FRAME;
        case 1 : return CTRL_ANGULAR_RATE;
        case 2 : return CTRL_ANGLE_ABSOLUTE_FRAME;
        default:
            ROS_ERROR_ONCE("Undefined axis input mode used. Check the config file.");
            return CTRL_ANGLE_ABSOLUTE_FRAME;
    }
}

void GimbalNode::reconfigureCallback(gremsy_base::ROSGremsyConfig &config, uint32_t level) {
    config_ = config;
}

int main(int argc, char **argv)
{
    // Init
    ros::init(argc, argv, "gremsy_base");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    GimbalNode n(nh, pnh);

    return 0;
}
