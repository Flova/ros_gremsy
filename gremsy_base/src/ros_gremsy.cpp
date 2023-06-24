#include <gremsy_base/ros_gremsy.h>



GimbalNode::GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    // Initialize dynamic-reconfigure
    dynamic_reconfigure::Server<gremsy_base::ROSGremsyConfig> server(pnh);
    dynamic_reconfigure::Server<gremsy_base::ROSGremsyConfig>::CallbackType f;
    f = boost::bind(&GimbalNode::reconfigureCallback, this, _1, _2);
    server.setCallback(f);

    // Init state variables
    goals_.vector.x = 0;
    goals_.vector.y = 0;
    goals_.vector.z = 0;

    // Advertive Publishers
    encoder_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/ros_gremsy/encoder", 1000);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/ros_gremsy/imu/data", 10);

    // Register Subscribers
    gimbal_goal_sub = nh.subscribe("/ros_gremsy/goals", 1, &GimbalNode::setGoalsCallback, this);

    // Define SDK objects
    serial_port_ = new Serial_Port(config_.device.c_str(), config_.baudrate);
    gimbal_interface_ = new Gimbal_Interface(serial_port_, 1, MAV_COMP_ID_ONBOARD_COMPUTER, Gimbal_Interface::MAVLINK_GIMBAL_V1);

    // Start ther serial interface and the gimbal SDK
    serial_port_->start();
    gimbal_interface_->start();

    // Wait for gimbal present
    while (!gimbal_interface_->present()) {
        ros::Duration(0.2).sleep();
    }

    ///////////////////
    // Config Gimbal //
    ///////////////////

    // Check if gimbal is on
    if(gimbal_interface_->get_gimbal_status().mode == Gimbal_Interface::GIMBAL_STATE_OFF)
    {
        // Turn on gimbal
        ROS_INFO("TURN_ON!\n");
        gimbal_interface_->set_gimbal_motor(Gimbal_Interface::TURN_ON);
    }

    // Wait until the gimbal is on
    while (gimbal_interface_->get_gimbal_status().mode < Gimbal_Interface::GIMBAL_STATE_ON)
    {
        ROS_WARN("Waiting for the gimbal to turn on!\n");
        ros::Duration(0.2).sleep();
    }

    // Set gimbal control modes
    if(config_.gimbal_mode == 1){
        gimbal_interface_->set_gimbal_follow_mode_sync();
    } else {
        gimbal_interface_->set_gimbal_lock_mode_sync();
    }

    // Configure the gimbal to send angles as encoder values
    gimbal_interface_->set_gimbal_encoder_type_send(false);

    ros::Timer poll_timer = nh.createTimer(
        ros::Duration(1/config_.state_poll_rate),
        &GimbalNode::gimbalStateTimerCallback, this);

    ros::Timer goal_timer = nh.createTimer(
        ros::Duration(1/config_.goal_push_rate),
        &GimbalNode::gimbalGoalTimerCallback, this);

    ros::spin();
}


void GimbalNode::gimbalStateTimerCallback(const ros::TimerEvent& event)
{
    // Publish Gimbal Encoder Values
    attitude<int16_t> encoder_values = gimbal_interface_->get_gimbal_encoder();

    geometry_msgs::Vector3Stamped encoder_ros_msg;
    encoder_ros_msg.header.stamp = ros::Time::now();
    encoder_ros_msg.vector.x = ((float) encoder_values.roll) * DEG_TO_RAD;
    encoder_ros_msg.vector.y = ((float) encoder_values.pitch) * DEG_TO_RAD;
    encoder_ros_msg.vector.z = ((float) encoder_values.yaw) * DEG_TO_RAD;

    encoder_pub.publish(encoder_ros_msg); 

    //// Publish Gimbal IMU (Currently this deadlocks)
    //Gimbal_Interface::imu_t imu_data = gimbal_interface_->get_gimbal_raw_imu();
    //
    // Create ROS message
    sensor_msgs::Imu imu_message;

    // Set header
    imu_message.header.stamp = encoder_ros_msg.header.stamp;
//
    //// Set accelaration data
    //imu_message.linear_acceleration.x = imu_data.accel.x;
    //imu_message.linear_acceleration.y = imu_data.accel.y;
    //imu_message.linear_acceleration.z = imu_data.accel.z;
//
    //// Set gyro data
    //imu_message.angular_velocity.x = imu_data.gyro.x;
    //imu_message.angular_velocity.y = imu_data.gyro.z;
    //imu_message.angular_velocity.z = imu_data.gyro.y;
    //
    //// Publish Camera Mount Orientation
    attitude<float> processed_gimbal_attitude = gimbal_interface_->get_gimbal_attitude();

    Eigen::Quaterniond imu_quaternion = convertYXZtoQuaternion(
        processed_gimbal_attitude.roll,
        -processed_gimbal_attitude.pitch,
        processed_gimbal_attitude.yaw);

    imu_message.orientation.x = imu_quaternion.x();
    imu_message.orientation.y = imu_quaternion.y();
    imu_message.orientation.z = imu_quaternion.z();
    imu_message.orientation.w = imu_quaternion.w();

    imu_pub.publish(imu_message);

    geometry_msgs::TransformStamped transform_imu;
    transform_imu.header.stamp = imu_message.header.stamp;
    transform_imu.header.frame_id = "imu_frame";
    transform_imu.child_frame_id = "map";
    transform_imu.transform.rotation = tf2::toMsg(imu_quaternion);
    bc_.sendTransform(transform_imu);
}


void GimbalNode::gimbalGoalTimerCallback(const ros::TimerEvent& event)
{
    gimbal_interface_->set_gimbal_rotation_sync(
        -goals_.vector.y * RAD_TO_DEG,
        goals_.vector.x * RAD_TO_DEG, 
        goals_.vector.z * RAD_TO_DEG);
}


void GimbalNode::setGoalsCallback(geometry_msgs::Vector3Stamped message)
{
    goals_ = message;
}


void GimbalNode::reconfigureCallback(gremsy_base::ROSGremsyConfig &config, uint32_t level) {
    config_ = config;
}


Eigen::Quaterniond GimbalNode::convertYXZtoQuaternion(double roll, double pitch, double yaw)
{
    Eigen::Quaterniond quat_abs(
                  Eigen::AngleAxisd(-DEG_TO_RAD * pitch, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(-DEG_TO_RAD * roll, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(DEG_TO_RAD * yaw, Eigen::Vector3d::UnitZ()));
    return quat_abs;
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
