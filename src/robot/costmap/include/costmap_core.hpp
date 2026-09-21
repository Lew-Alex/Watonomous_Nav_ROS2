#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>


namespace robot
{

class CostmapCore {
public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void intialize(double resolution, double width, double height,
                   double inflation_radius, int max_cost);

    void setObstacle(double angle, double range);

    void inflate();

    double resolution_;
    int    width_cells_;
    int    height_cells_;
    double inflation_radius_ = 2.0;
    uint8_t max_cost_        = 100;

    std::vector<int8_t>* grid();

private:
    rclcpp::Logger logger_;
    std::vector<int8_t> grid_;
    
    void propagateCost(int x, int y);

    uint32_t getGrid(int x, int y);
    void setGrid(double x, double y, uint8_t val); // x, y in meters
};

}  

#endif  