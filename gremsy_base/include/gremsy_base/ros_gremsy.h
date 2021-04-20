#include <unistd.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <gremsy_base/ROSGremsyConfig.h>
#include <cmath>
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include "gimbal_interface.h"
#include "serial_port.h"

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
    // Calback to set a new gimbal position
    void setGoalsCallback(geometry_msgs::Vector3Stamped message);
    // Converts
    sensor_msgs::Imu convertImuMavlinkMessageToROSMessage(mavlink_raw_imu_t message);
    Eigen::Quaterniond convertXZYtoQuaternion(double roll, double pitch, double yaw);
    // Maps integer mode
    control_gimbal_axis_input_mode_t convertIntToAxisInputMode(int mode);
    // Maps integer mode
    control_gimbal_mode_t convertIntGimbalMode(int mode);

    // Gimbal SDK
    Gimbal_Interface* gimbal_interface_;
    // Serial Interface
    Serial_Port* serial_port_;
    // Current config
    gremsy_base::ROSGremsyConfig config_;
    // Publishers
    ros::Publisher
        imu_pub,
        encoder_pub,
        mount_orientation_incl_global_yaw,
        mount_orientation_incl_local_yaw;
    // Subscribers
    ros::Subscriber gimbal_goal_sub;
};
