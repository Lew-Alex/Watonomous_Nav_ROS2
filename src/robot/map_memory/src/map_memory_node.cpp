#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

    this->declare_parameter("resolution", 0.1);           // metres per cell
    this->declare_parameter("width_cells", 300);          // 300 cells at 0.1 m is 30 m
    this->declare_parameter("height_cells", 300);
    this->declare_parameter("origin_x", -15.0);           // world coordinate of cell (0,0)
    this->declare_parameter("origin_y", -15.0);
    this->declare_parameter("update_period_seconds", 1);  // how often the global map is republished
    this->declare_parameter("move_threshold", 1.5);       // metres of travel before integrating a scan

    resolution            = this->get_parameter("resolution").as_double();
    width_cells           = this->get_parameter("width_cells").as_int();
    height_cells          = this->get_parameter("height_cells").as_int();
    origin_x              = this->get_parameter("origin_x").as_double();
    origin_y              = this->get_parameter("origin_y").as_double();
    update_period_seconds = this->get_parameter("update_period_seconds").as_int();
    move_threshold        = this->get_parameter("move_threshold").as_double();

    map_memory_.initialize(resolution, width_cells, height_cells, origin_x, origin_y);

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);


    timer_ = this->create_wall_timer(
        std::chrono::seconds(update_period_seconds), std::bind(&MapMemoryNode::updateMap, this));

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
    if (distance >= move_threshold) {
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
