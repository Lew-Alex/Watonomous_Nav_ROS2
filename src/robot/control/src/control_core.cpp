#include "control_core.hpp"

namespace
{
constexpr double kTargetEpsilon = 1e-6;  // squared metres; target effectively on top of us
}

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

int ControlCore::lookAheadIdx(nav_msgs::msg::Path::SharedPtr path, nav_msgs::msg::Odometry::SharedPtr odom){
    if (path == nullptr || odom == nullptr || path->poses.empty()){
        return -1;
    }

    const double rx = odom->pose.pose.position.x;
    const double ry = odom->pose.pose.position.y;

    auto dist2 = [&](const geometry_msgs::msg::PoseStamped & ps) {
        const double dx = ps.pose.position.x - rx;
        const double dy = ps.pose.position.y - ry;
        return hypot(dx, dy);
    };

    int closest = 0;
    double best = dist2(path->poses[0]);

    for (size_t i = 1; i < path->poses.size(); i++){
        const double d2 = dist2(path->poses[i]);
        if (d2 < best){
            best = d2;
            closest = static_cast<int>(i);
        }
    }

    // closest is the index of the closest point
    for (size_t i = closest; i < path->poses.size(); i++){
        const double d2 = dist2(path->poses[i]);
        if (d2 > look_ahead_dis_){
            return i;
        }
    }

    return static_cast<int>(path->poses.size()) - 1;

}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion & q){
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(geometry_msgs::msg::PoseStamped* target, nav_msgs::msg::Odometry::SharedPtr odom){
    const double yaw = extractYaw(odom->pose.pose.orientation);

    const double dx = target->pose.position.x - odom->pose.pose.position.x;
    const double dy = target->pose.position.y - odom->pose.pose.position.y;

    const double local_x =  dx * std::cos(yaw) + dy * std::sin(yaw);
    const double local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);

    const double l2 = local_x * local_x + local_y * local_y;
    if (l2 < kTargetEpsilon){
        return geometry_msgs::msg::Twist(); // target is basically on top of us
    }

    // angle from the nose to the target, positive = target is to our left
    const double alpha = std::atan2(local_y, local_x);

    const double curvature = 2.0 * local_y / l2;

    const double linear_error = hypot(dx, dy);
    const double power = linear_error * speed_gain_;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x  = std::clamp(power, min_speed_, max_speed_);
    cmd_vel.angular.z = cmd_vel.linear.x * curvature;

    return cmd_vel;

}



} 
