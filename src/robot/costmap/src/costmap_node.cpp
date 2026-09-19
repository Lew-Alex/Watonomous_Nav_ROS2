#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10,
        std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

    // status_pub_ = this->create_publisher<std_msgs::msg::String>("/testing_stuff", 10);
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    this->declare_parameter("resolution", 0.1); // Meters / cell
    this->declare_parameter("width", 24.0); // Width in Meters
    this->declare_parameter("height", 24.0); // Height in Meters

    resolution = this->get_parameter("resolution").as_double();
    width = this->get_parameter("width").as_double();
    height = this->get_parameter("height").as_double();
}

void CostmapNode::publishCostMap(const sensor_msgs::msg::LaserScan::SharedPtr scan){
    nav_msgs::msg::OccupancyGrid map;

    map.header.stamp = scan->header.stamp;
    map.header.frame_id = scan->header.frame_id;

    map.info.resolution = costmap_.resolution_;
    map.info.width      = costmap_.width_cells_;
    map.info.height     = costmap_.height_cells_;

    // Robots center is origin
    map.info.origin.position.x = -costmap_.width_cells_  * costmap_.resolution_ / 2.0;
    map.info.origin.position.y = -costmap_.height_cells_ * costmap_.resolution_ / 2.0;
    map.info.origin.orientation.w = 1.0;

    map.data = (*costmap_.grid());   // vector<int8_t> -> vector<int8_t>, one copy

    costmap_pub_->publish(map);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
    // auto msg = std_msgs::msg::String();
    // msg.data = "First scan has distance " + std::to_string(scan->ranges[int(scan->ranges.size() / 2)]);
    // status_pub_->publish(msg);
    

    costmap_.intialize(resolution, width, height);

    double SCAN_MAX = scan->range_max;
    double SCAN_MIN = scan->range_min;
    

    // Look through all the scans
    for (size_t i = 0; i < scan->ranges.size(); i++){
        double scan_angle = scan->angle_min + i * scan->angle_increment;
        double scan_range = scan->ranges[i];

        if (scan_range < SCAN_MAX && scan_range > SCAN_MIN){
            // Proper scan
            costmap_.setObstacle(scan_angle, scan_range);
        }
    }
    // INflate obstacles
    costmap_.inflate();

    // Obstacles set
    publishCostMap(scan);
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}