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
        ros::Duration(0.2).sleep();
    }

    // Set gimbal control modes
    if(config_.gimbal_mode == 1){
        gimbal_interface_->set_gimbal_follow_mode_sync();
    } else {
        gimbal_interface_->set_gimbal_lock_mode_sync();
    }

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
    encoder_ros_msg.vector.x = ((float) encoder_values.roll);
    encoder_ros_msg.vector.y = ((float) encoder_values.pitch);
    encoder_ros_msg.vector.z = ((float) encoder_values.yaw);

    encoder_pub.publish(encoder_ros_msg); 
}




void GimbalNode::gimbalGoalTimerCallback(const ros::TimerEvent& event)
{

    gimbal_interface_->set_gimbal_rotation_sync(goals_.vector.y,goals_.vector.x, goals_.vector.z);

}

void GimbalNode::setGoalsCallback(geometry_msgs::Vector3Stamped message)
{
    goals_ = message;
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
