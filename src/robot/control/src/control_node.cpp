#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {

    this->declare_parameter("control_period_ms", 100);
    this->declare_parameter("arrival_tolerance", 0.5);
    this->declare_parameter("look_ahead_dis", 2.0);
    this->declare_parameter("speed_gain", 1.5);
    this->declare_parameter("min_speed", 0.75);
    this->declare_parameter("max_speed", 3.0);

    control_period_ms  = this->get_parameter("control_period_ms").as_int();
    arrival_tolerance  = this->get_parameter("arrival_tolerance").as_double();

    control_.look_ahead_dis_ = this->get_parameter("look_ahead_dis").as_double();
    control_.speed_gain_     = this->get_parameter("speed_gain").as_double();
    control_.min_speed_      = this->get_parameter("min_speed").as_double();
    control_.max_speed_      = this->get_parameter("max_speed").as_double();

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(control_period_ms), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg){
    current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    robot_odom_ = msg;
}

void ControlNode::controlLoop(){
    if (current_path_ == nullptr || robot_odom_ == nullptr){
        return;
    }

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        return;
    }

    auto cmd_vel = control_.computeVelocity(&*lookahead_point, robot_odom_);

    const auto & last = current_path_->poses.back().pose.position;
    const double dx = last.x - robot_odom_->pose.pose.position.x;
    const double dy = last.y - robot_odom_->pose.pose.position.y;

    if (std::hypot(dx, dy) < arrival_tolerance){
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // publish zeros, don't just go quiet
        return;
    }

 
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    int idx = control_.lookAheadIdx(current_path_, robot_odom_);

    if (idx < 0){
        return std::nullopt;
    }

    return current_path_->poses[idx];
    
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
