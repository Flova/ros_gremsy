#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ros_gremsy/ROSGremsyConfig.h>
#include <boost/bind.hpp>

class GimbalNode
{
public:
    // Params: (public node handler (for e.g. callbacks), private node handle (for e.g. dynamic reconfigure))
    GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    // Dynamic reconfigure callback
    void callbackRC(ros_gremsy::WhiteBalancerConfig &config, uint32_t level);
private:
    int temp = 400;
};

GimbalNode::GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    // Dynamic reconfigure stuff
    dynamic_reconfigure::Server<ros_gremsy::WhiteBalancerConfig> server(pnh);
    dynamic_reconfigure::Server<ros_gremsy::WhiteBalancerConfig>::CallbackType f;
    f = boost::bind(&GimbalNode::callbackRC, this, _1, _2);
    server.setCallback(f);

    std::string ROS_output_topic, ROS_input_topic;
    if (pnh.getParam("/gimbal/ROS_output_topic", ROS_output_topic)) {
        GimbalNode::pub = it.advertise(ROS_output_topic, 1);
    } else {
        ROS_ERROR("No output topic set");
        exit(2);
    }
    image_transport::Subscriber sub;
    if (pnh.getParam("/gimbal/ROS_input_topic", ROS_input_topic)) {
         sub = it.subscribe(ROS_input_topic, 1, &GimbalNode::imageCallback, this);
    } else {
        ROS_ERROR("No input topic set");
        exit(2);
    }

    ros::spin();
}

void GimbalNode::callbackRC(white_balancer::WhiteBalancerConfig &config, uint32_t level) {
    // Set color temperature
    //WhiteBalancer::set_temp(config.temp);
    // Set timestamp delay
    //WhiteBalancer::set_delay(config.timestamp_offset);
}

int main(int argc, char **argv)
{
    // Init
    ros::init(argc, argv, "ros_gremsy");
    ros::NodeHandle nh;
    GimbalNode n(nh, nh);

    return 0;
}
