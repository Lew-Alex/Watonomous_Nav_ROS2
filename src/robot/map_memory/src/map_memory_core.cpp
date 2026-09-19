#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {

    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    global_map_->info.resolution = 0.1f;
    global_map_->info.width  = 300;         // 30 m
    global_map_->info.height = 300;
    global_map_->info.origin.position.x = -15.0;
    global_map_->info.origin.position.y = -15.0;
    global_map_->header.frame_id = "sim_world";

    global_map_->data.assign(global_map_->info.width * global_map_->info.height, -1); // Starts unknown
}



nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::intergrateMap(nav_msgs::msg::OccupancyGrid::SharedPtr new_map, double x, double y, double yaw){
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    double global_origin_x = global_map_->info.origin.position.x;
    double global_origin_y = global_map_->info.origin.position.y;
    double global_res = global_map_->info.resolution;

    double map_origin_x = new_map->info.origin.position.x;
    double map_origin_y = new_map->info.origin.position.y;
    double map_res = new_map->info.resolution;


    for (int x_i = 0; x_i < new_map->info.width; x_i++){
        for (int y_i = 0; y_i < new_map->info.height; y_i++){

            int idx = y_i * new_map->info.width + x_i;

            double x_robot = map_origin_x + x_i * map_res;
            double y_robot = map_origin_y + y_i * map_res;

            double x_world = x + x_robot * cos_yaw - y_robot * sin_yaw;
            double y_world = y + x_robot * sin_yaw + y_robot * cos_yaw;

            
            const int global_x_i = static_cast<int>(std::round((x_world - global_origin_x) / global_res));
            const int global_y_i = static_cast<int>(std::round((y_world - global_origin_y) / global_res));

            if (global_x_i < 0 || global_x_i >= static_cast<int>(global_map_->info.width) || global_y_i < 0 || global_y_i >= static_cast<int>(global_map_->info.height)) {
                continue;
            }

            int global_idx = static_cast<size_t>(global_y_i) * global_map_->info.width + global_x_i;

            if (global_map_->data[global_idx] < new_map->data[idx]){
                global_map_->data[global_idx] = new_map->data[idx];
            }

            

        }
    }
    global_map_->header.stamp = new_map->header.stamp;
    // RCLCPP_INFO(logger_, "global map frame_id: %s", global_map_->header.frame_id.c_str());


    return global_map_;
}

} 


