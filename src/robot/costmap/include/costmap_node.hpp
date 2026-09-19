#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

private:
    // Parameters
    double resolution;
    double width;
    double height;

    robot::CostmapCore costmap_;

    // Lidar Sub
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    void publishCostMap(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};

#endif 