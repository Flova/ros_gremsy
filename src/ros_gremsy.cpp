#include <ros_gremsy/ros_gremsy.h>

GimbalNode::GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    // Initialize dynamic-reconfigure
    dynamic_reconfigure::Server<ros_gremsy::ROSGremsyConfig> server(pnh);
    dynamic_reconfigure::Server<ros_gremsy::ROSGremsyConfig>::CallbackType f;
    f = boost::bind(&GimbalNode::reconfigureCallback, this, _1, _2);
    server.setCallback(f);

    // Advertive Publishers
    imu_pub = pnh.advertise<sensor_msgs::Imu>("imu/data", 10);
    encoder_pub = pnh.advertise<geometry_msgs::Vector3Stamped>("encoder", 10);
    mount_orientation_incl_global_yaw = pnh.advertise<geometry_msgs::Quaternion>("mount_orientation_global_yaw", 10);
    mount_orientation_incl_local_yaw = pnh.advertise<geometry_msgs::Quaternion>("mount_orientation_local_yaw", 10);


    // Register Subscribers
    gimbal_goal_sub = pnh.subscribe("goals", 1, &GimbalNode::setGoalsCallback, this);

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
    ros::Time timestamp = ros::Time::now();

    // Publish Gimbal IMU
    mavlink_raw_imu_t imu_mav = gimbal_interface_->get_gimbal_raw_imu();
    imu_mav.time_usec = gimbal_interface_->get_gimbal_time_stamps().raw_imu; // TODO implement rostime
    sensor_msgs::Imu imu_ros_mag = convertImuMavlinkMessageToROSMessage(imu_mav);
    imu_pub.publish(imu_ros_mag);

    // Publish Gimbal Encoder Values
    mavlink_mount_status_t mount_status = gimbal_interface_->get_gimbal_mount_status();
    geometry_msgs::Vector3Stamped encoder_ros_msg;
    encoder_ros_msg.header.stamp = timestamp;
    encoder_ros_msg.vector.x = (float) mount_status.pointing_b * DEG_TO_RAD;
    encoder_ros_msg.vector.y = (float) mount_status.pointing_a * DEG_TO_RAD;
    encoder_ros_msg.vector.z = (float) mount_status.pointing_c * DEG_TO_RAD;
    // encoder_ros_msg.header TODO frame
    encoder_pub.publish(encoder_ros_msg);

    // Publish yaw motor transform
    geometry_msgs::TransformStamped transform_yaw_motor;
    transform_yaw_motor.header.stamp = timestamp;
    transform_yaw_motor.header.frame_id = "gimbal_mount";
    transform_yaw_motor.child_frame_id = "yaw_arm";
    tf2::Quaternion q_yaw_rot;
    q_yaw_rot.setRPY(0, 0, encoder_ros_msg.vector.z);
    tf2::convert(q_yaw_rot, transform_yaw_motor.transform.rotation);
    transform_yaw_motor.transform.translation.z = -0.05;
    bc_.sendTransform(transform_yaw_motor);

    // Publish roll motor transform
    geometry_msgs::TransformStamped transform_roll_motor;
    transform_roll_motor.header.stamp = timestamp;
    transform_roll_motor.header.frame_id = "yaw_arm";
    transform_roll_motor.child_frame_id = "roll_arm";
    tf2::Quaternion q_roll_rot;
    q_roll_rot.setRPY(encoder_ros_msg.vector.x, 0, 0);
    tf2::convert(q_roll_rot, transform_roll_motor.transform.rotation);
    transform_roll_motor.transform.translation.x = -0.18;
    transform_roll_motor.transform.translation.z = -0.15;
    bc_.sendTransform(transform_roll_motor);

    // Publish pitch motor transform
    geometry_msgs::TransformStamped transform_pitch_motor;
    transform_pitch_motor.header.stamp = timestamp;
    transform_pitch_motor.header.frame_id = "roll_arm";
    transform_pitch_motor.child_frame_id = "pitch_arm";
    tf2::Quaternion q_pitch_rot;
    q_pitch_rot.setRPY(0, encoder_ros_msg.vector.y, 0);
    tf2::convert(q_pitch_rot, transform_pitch_motor.transform.rotation);
    transform_pitch_motor.transform.translation.x = 0.18;
    transform_pitch_motor.transform.translation.y = 0.12;
    bc_.sendTransform(transform_pitch_motor);

    // Publish static tf to camera mount
    geometry_msgs::TransformStamped transform_camera_mount;
    transform_camera_mount.header.stamp = timestamp;
    transform_camera_mount.header.frame_id = "pitch_arm";
    transform_camera_mount.child_frame_id = "camera_mount";
    transform_camera_mount.transform.translation.y = -0.12;
    bc_.sendTransform(transform_camera_mount);


    // Get Mount Orientation
    mavlink_mount_orientation_t mount_orientation = gimbal_interface_->get_gimbal_mount_orientation();

    // Publish Camera Mount Orientation in global frame (drifting)
    tf2::Quaternion quat_abs;
    quat_abs.setRPY(
        DEG_TO_RAD * mount_orientation.roll,
        DEG_TO_RAD * mount_orientation.pitch,
        DEG_TO_RAD * mount_orientation.yaw_absolute);
    quat_abs.normalize();

    geometry_msgs::Quaternion quat_abs_msg;
    tf2::convert(quat_abs , quat_abs_msg);
    mount_orientation_incl_global_yaw.publish(quat_abs_msg);

    // Publish transform for Camera Mount Orientation
    geometry_msgs::TransformStamped transform_camera_mount_imu;
    transform_camera_mount_imu.header.stamp =timestamp;
    transform_camera_mount_imu.header.frame_id = "gimbal_imu";
    transform_camera_mount_imu.child_frame_id = "camera_mount";
    transform_camera_mount_imu.transform.rotation = quat_abs_msg;
    bc_.sendTransform(transform_camera_mount_imu);


    // Publish Camera Mount Orientation in local frame (yaw relative to vehicle)
    tf2::Quaternion quat_loc;
    quat_loc.setRPY(
        DEG_TO_RAD * mount_orientation.roll,
        DEG_TO_RAD * mount_orientation.pitch,
        DEG_TO_RAD * mount_orientation.yaw);
    quat_loc.normalize();

    geometry_msgs::Quaternion quat_loc_msg;
    tf2::convert(quat_loc , quat_loc_msg);
    mount_orientation_incl_local_yaw.publish(quat_loc_msg);

    // Publish transform for Camera Mount Orientation with yaw relative to mount
    geometry_msgs::TransformStamped transform_camera_mount_imu_loc_yaw;
    transform_camera_mount_imu_loc_yaw.header.stamp =timestamp;
    transform_camera_mount_imu_loc_yaw.header.frame_id = "gimbal_imu_fixed_yaw";
    transform_camera_mount_imu_loc_yaw.child_frame_id = "camera_mount";
    transform_camera_mount_imu_loc_yaw.transform.rotation = quat_loc_msg;
    bc_.sendTransform(transform_camera_mount_imu_loc_yaw);
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

void GimbalNode::reconfigureCallback(ros_gremsy::ROSGremsyConfig &config, uint32_t level) {
    config_ = config;
}

int main(int argc, char **argv)
{
    // Init
    ros::init(argc, argv, "ros_gremsy");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    GimbalNode n(nh, pnh);

    return 0;
}
