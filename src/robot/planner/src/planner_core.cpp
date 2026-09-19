#include "planner_core.hpp"
#include <algorithm>    // std::reverse
#include <cmath>        // std::floor, std::abs, std::sqrt
#include <cstddef>      // std::size_t
#include <cstdint>      // int8_t
#include <queue>        // std::priority_queue
#include <unordered_map>
#include <unordered_set>
#include <vector>


namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}


void PlannerCore::plan(nav_msgs::msg::Path* path, nav_msgs::msg::OccupancyGrid* map, geometry_msgs::msg::PointStamped* goal, geometry_msgs::msg::Pose* odom){
    path->poses.clear();

    const int width = static_cast<int>(map->info.width);
    const int height = static_cast<int>(map->info.height);
    const double resolution = map->info.resolution;
    const double origin_x = map->info.origin.position.x;
    const double origin_y = map->info.origin.position.y;


    auto to_cell = [&](double wx, double wy) {
        return CellIndex(
            static_cast<int>(std::floor((wx - origin_x) / resolution)),
            static_cast<int>(std::floor((wy - origin_y) / resolution)));
    };

    CellIndex start = to_cell(odom->position.x, odom->position.y);
    CellIndex end = to_cell(goal->point.x, goal->point.y);

    RCLCPP_INFO(logger_, "start cell (%d,%d) world (%.2f,%.2f) | goal cell (%d,%d) world (%.2f,%.2f)", start.x, start.y, odom->position.x, odom->position.y, end.x, end.y, goal->point.x, goal->point.y);

    
    auto blocked = [&](const CellIndex & c) {
        if (c.x < 0 || c.x >= width || c.y < 0 || c.y >= height){
            return true; 
        }
        const int8_t cost = map->data[static_cast<size_t>(c.y) * width + c.x];
        return cost >= 25; // 50 threshold for if its blocked or not
    };

    struct Step { int dx; int dy; double cost; };
    const std::vector<Step> neighbours = {
        { 1,  0, 1.0},
        {-1,  0, 1.0},
        { 0,  1, 1.0},
        { 0, -1, 1.0},

        { 1, 1, 1.41},
        { 1, -1, 1.41},
        { -1, -1, 1.41},
        { -1, 1, 1.41},
    };

    auto calc_h = [&](const CellIndex & c) {
        const double dx = std::abs(static_cast<double>(end.x - c.x));
        const double dy = std::abs(static_cast<double>(end.y - c.y));
        return hypot(dx, dy);
    };

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;

    open.push(AStarNode(start, calc_h(start), 0.0)); // Cell, g_score, f_score
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_set<CellIndex, CellIndexHash> past;
    std::unordered_map<CellIndex, double, CellIndexHash> best_g;
    best_g[start] = 0.0;




    // Main loop to find pathb
    bool found = false;
    while (!open.empty()){
        const AStarNode top = open.top();
        open.pop();

        const CellIndex current = top.index;
        const double g_score_  = top.g_score;

        if (past.count(current)) {
            continue;
        } else {
            past.insert(current);
        }
        


        if (current == end) {
            found = true;
            break;
        }

        // Calc around the current point
        for (Step s: neighbours){
            CellIndex next(current.x + s.dx, current.y + s.dy);

            if (blocked(next) || past.count(next)) {
                continue;
            }

            double cur_g_score = g_score_ + s.cost;

            auto it = best_g.find(next);
            if (it != best_g.end() && cur_g_score >= it->second){
                continue;
            }                                                          


            best_g[next] = cur_g_score;
            came_from[next] = current;
            open.push(AStarNode(next, calc_h(next) + cur_g_score, cur_g_score));
        }
    }
    // The end!
    if (!found){
        RCLCPP_WARN(logger_, "no path from (%d,%d) to (%d,%d)",
                    start.x, start.y, end.x, end.y);
        return;   // poses stays empty, which tells control to stop

    }

    std::vector<CellIndex> cells;
    for (CellIndex c = end; ; c = came_from.at(c)) {
        cells.push_back(c);
        if (c == start) {
            break;
        }
    }
    std::reverse(cells.begin(), cells.end());

    auto to_world_x = [&](int x) { return origin_x + x * resolution; };
    auto to_world_y = [&](int y) { return origin_y + y * resolution; };

    for (const CellIndex & c : cells) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path->header;
        ps.pose.position.x = to_world_x(c.x);
        ps.pose.position.y = to_world_y(c.y);
        ps.pose.position.z = 0.0;
        path->poses.push_back(ps);
    }


}


} 
