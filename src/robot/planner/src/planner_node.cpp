#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

// Call back methods
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Goal defined!");
    goal_ = (*msg);
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    current_map_ = (*msg);
    // New map replan
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL){
        planPath();
    }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    odom_ = msg->pose.pose;
}

void PlannerNode::timerCallback(){
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }

}

bool PlannerNode::goalReached(){
    double dx = goal_.point.x - odom_.position.x;
    double dy = goal_.point.y - odom_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5;

}


void PlannerNode::planPath(){
    if (current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map!");
        return;
    }

    // A* Implementation (pseudo-code)
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = current_map_.header.frame_id;

    
    planner_.plan(&path, &current_map_, &goal_, &odom_);

    if (path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "No path found, keeping the previous one");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Published Map!");
    path_pub_->publish(path);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
