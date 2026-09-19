#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    int lookAheadIdx(nav_msgs::msg::Path::SharedPtr path, nav_msgs::msg::Odometry::SharedPtr odom);

    geometry_msgs::msg::Twist computeVelocity(geometry_msgs::msg::PoseStamped* target, nav_msgs::msg::Odometry::SharedPtr odom);


  
  private:
    rclcpp::Logger logger_;

    double extractYaw(const geometry_msgs::msg::Quaternion & q);

    double look_ahead_dis_ = 1.0;
    double linear_speed_ = 1.0;
};

} 

#endif 
