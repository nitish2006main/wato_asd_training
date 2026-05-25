#include "planner_core.hpp"
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <unordered_map>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

nav_msgs::msg::Path PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map, 
      const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::PointStamped& goal)
{
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> openlist; //to be able to get the lowest f_score
    std::unordered_set<CellIndex, CellIndexHash> open_set; //to be able to check if node is in the list
    std::unordered_map<CellIndex, int, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_back;
    std::unordered_set<CellIndex, CellIndexHash> closedlist;
    nav_msgs::msg::Path path;
    path.header.frame_id = "sim_world";
    geometry_msgs::msg::PoseStamped pose; 

    int x_grid = (int)((start.position.x - map.info.origin.position.x)/map.info.resolution);
    int y_grid = (int)((start.position.y - map.info.origin.position.y)/map.info.resolution);
    CellIndex startpos(x_grid, y_grid);

    int x_goal = (int)((goal.point.x - map.info.origin.position.x)/map.info.resolution);
    int y_goal = (int)((goal.point.y - map.info.origin.position.y)/map.info.resolution);
    CellIndex endpos(x_goal, y_goal);

    if (x_grid < 0 || x_grid >= (int)map.info.width || y_grid < 0 || y_grid >= (int)map.info.height) {
        RCLCPP_WARN(logger_, "Start position out of map bounds");
        return path;
    }
    if (x_goal < 0 || x_goal >= (int)map.info.width || y_goal < 0 || y_goal >= (int)map.info.height) {
        RCLCPP_WARN(logger_, "Goal position out of map bounds");
        return path;
    }

    g_score[startpos] = 0;
    openlist.push(AStarNode(startpos, g_score[startpos] + sqrt(pow(x_goal - x_grid, 2) 
    + pow(y_goal - y_grid, 2))));
    open_set.insert(startpos);

    int dx[8] = {0, 0, 1, -1, 1, 1, -1, -1};
    int dy[8] = {1, -1, 0, 0, 1, -1, 1, -1};

    while(!openlist.empty())
    {
        AStarNode current = openlist.top();
        openlist.pop();

        if (closedlist.count(current.index)) continue;

        open_set.erase(current.index);
        closedlist.insert(current.index);

        if (current.index == endpos)
        {
            CellIndex current_cell = endpos;
            while(current_cell != startpos)
            {
                double world_x = current_cell.x * map.info.resolution + map.info.origin.position.x;
                double world_y = current_cell.y * map.info.resolution + map.info.origin.position.y;
                pose.header.frame_id = "sim_world";
                pose.pose.position.x = world_x;
                pose.pose.position.y = world_y;
                path.poses.push_back(pose);
                current_cell = came_back[current_cell];
            }
            std::reverse(path.poses.begin(), path.poses.end());
            return path;
        }
        
        for (int i = 0; i < 8; i++)
        {
            int neighbor_x = current.index.x + dx[i];
            int neighbor_y = current.index.y + dy[i];
            CellIndex neighbor(neighbor_x, neighbor_y);
            if ((neighbor_x < 0 || neighbor_x >= (int)(map.info.width) 
                || neighbor_y < 0 || neighbor_y >= (int)(map.info.height)) ||
                map.data[neighbor.y * map.info.width + neighbor.x] >= 100 || closedlist.count(neighbor))
            {
                continue;
            }

            int move_cost = (dx[i] != 0 && dy[i] != 0) ? 14 : 10;
            int cell_cost = map.data[neighbor.y * map.info.width + neighbor.x];
            int new_g = g_score[current.index] + move_cost + cell_cost;

            bool not_visited = (open_set.count(neighbor) == 0);
            if (not_visited || new_g < g_score[neighbor])
            {
                g_score[neighbor] = new_g;
                came_back[neighbor] = current.index;
                int h = (int)(sqrt(pow(neighbor_x - x_goal, 2) + pow(neighbor_y - y_goal, 2)) * 10);
                openlist.push(AStarNode(neighbor, new_g + h));
                open_set.insert(neighbor);
            }
        }
    }
    return path;
}

} 
