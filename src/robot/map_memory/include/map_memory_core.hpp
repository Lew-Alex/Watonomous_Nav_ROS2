#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initialize(double resolution, int width_cells, int height_cells,
                    double origin_x, double origin_y);

    nav_msgs::msg::OccupancyGrid::SharedPtr intergrateMap(nav_msgs::msg::OccupancyGrid::SharedPtr new_map, double x, double y, double yaw);

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_ = nullptr;

};

}  

#endif  
