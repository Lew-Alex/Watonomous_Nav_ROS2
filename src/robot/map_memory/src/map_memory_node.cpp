#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);


    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

}


void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    auto & q = msg->pose.pose.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    robot_x_ = x;
    robot_y_ = y;
    robot_yaw_ = yaw;


    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= 1.5) { // Update distance threshold to not have a magic number ***
        last_x = x;
        last_y = y;
        update_map_ = true;
    }

}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    latest_map_ = msg;
}

void MapMemoryNode::updateMap(){
    if (latest_map_ != nullptr && update_map_){
        update_map_ = false;
        nav_msgs::msg::OccupancyGrid::SharedPtr global_map_ = map_memory_.intergrateMap(latest_map_, robot_x_, robot_y_, robot_yaw_);
        map_pub_->publish(*global_map_);
    }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
