#include "costmap_core.hpp"
#include <cmath>

namespace
{
constexpr uint8_t kFreeCost = 0;  // a cell nothing has been seen in
}

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}
    


void CostmapCore::intialize(double resolution, double width, double height,
                            double inflation_radius, int max_cost){
    this->resolution_       = resolution;
    this->width_cells_      = static_cast<int>(width / resolution);
    this->height_cells_     = static_cast<int>(height / resolution);
    this->inflation_radius_ = inflation_radius;
    this->max_cost_         = static_cast<uint8_t>(max_cost);

    grid_.assign(static_cast<size_t>(width_cells_) * static_cast<size_t>(height_cells_), kFreeCost);
}



void CostmapCore::setObstacle(double angle, double range){
    // Both follow values in meters
    double obstacle_x = range * cos(angle);
    double obstacle_y = range * sin(angle);

    this->setGrid(obstacle_x, obstacle_y, max_cost_);
}

void CostmapCore::inflate(){
    for (int y = 0; y < height_cells_; y++){
        for (int x = 0; x < width_cells_; x++){
            uint32_t idx = getGrid(x, y);
            if (grid_[idx] == max_cost_){
                propagateCost(x, y);
            }
        }
    }
}

void CostmapCore::propagateCost(int x, int y){
    int reach = std::ceil(inflation_radius_ / resolution_);

    for (int dy = -reach; dy <= reach; dy++){
        for (int dx = -reach; dx <= reach; dx++){
            if (dy == 0 && dx == 0){
                continue;
            }

            int x_ = x + dx;
            int y_ = y + dy;

            if (x_ < 0 || x_ >= width_cells_ ||
                y_ < 0 || y_ >= height_cells_){
                continue;
            }

            double distance = std::hypot(dx, dy) * resolution_;

            if (distance > inflation_radius_){
                continue;
            }

            int cost = static_cast<uint8_t>(max_cost_ * (1.0 - distance / inflation_radius_));

            // Update cost if its higher
            if (grid_[getGrid(x_, y_)] < cost) grid_[getGrid(x_, y_)] = cost;
        }
    }

}

void CostmapCore::setGrid(double x, double y, uint8_t val){ // x and y in meters
    int16_t x_ = floor(x / resolution_) + width_cells_ / 2;
    int16_t y_ = floor(y / resolution_) + height_cells_ / 2;

    if (x_ < 0 || y_ < 0 || x_ >= width_cells_ || y_ >= height_cells_){
        return;
    }


    uint32_t idx = y_ * width_cells_ + x_; // In the vector its stored by rows(so the first x values are all in the same row)
    grid_[idx] = val;
}

uint32_t CostmapCore::getGrid(int x, int y){ // x and y in meters
    uint32_t idx = y * width_cells_ + x; // In the vector its stored by rows(so the first x values are all in the same row)
    return idx;
}


std::vector<int8_t>* CostmapCore::grid(){
    return &grid_;
}

}