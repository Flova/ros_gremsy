#include <boost/bind.hpp>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gremsy_base/ROSGremsyConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <unistd.h>
#include "gimbal_interface.h"
#include "serial_port.h"
#include <signal.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

class GimbalNode
{
public:
    // Params: (public node handler (for e.g. callbacks), private node handle (for e.g. dynamic reconfigure))
    GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh);
private:
    // Dynamic reconfigure callback
    void reconfigureCallback(gremsy_base::ROSGremsyConfig &config, uint32_t level);
    // Timer which checks for new infomation regarding the gimbal
    void gimbalStateTimerCallback(const ros::TimerEvent& event);
    // Timer which sets the gimbal goals
    void gimbalGoalTimerCallback(const ros::TimerEvent& event);
    // Calback to set a new gimbal position
    void setGoalsCallback(geometry_msgs::Vector3Stamped message);
    // Converter
    Eigen::Quaterniond convertYXZtoQuaternion(double roll, double pitch, double yaw);

    // Gimbal SDK
    Gimbal_Interface* gimbal_interface_;
    // Serial Interface
    Serial_Port* serial_port_;
    // Current config
    gremsy_base::ROSGremsyConfig config_;
    // Publishers
    ros::Publisher encoder_pub, imu_pub;
    // Subscribers
    ros::Subscriber gimbal_goal_sub;

    // TF Broadcasters
    tf2_ros::TransformBroadcaster bc_;

    // Value store
    geometry_msgs::Vector3Stamped goals_;


};
