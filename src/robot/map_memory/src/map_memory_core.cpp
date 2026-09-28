#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap(){
  return global_map_;
}

void MapMemoryCore::initializeGlobalMap() {
  global_map_.info.width = GLOBAL_WIDTH;
  global_map_.info.height = GLOBAL_HEIGHT;
  global_map_.info.resolution = RESOLUTION;
  global_map_.info.origin.position.x = -(GLOBAL_WIDTH * RESOLUTION) / 2.0;
  global_map_.info.origin.position.y = -(GLOBAL_HEIGHT * RESOLUTION) / 2.0;
  global_map_.header.frame_id = "sim_world";
  global_map_.data.assign(GLOBAL_WIDTH * GLOBAL_HEIGHT, -1);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x, double robot_y, double robot_yaw) {
  if (global_map_.data.empty()) {
    initializeGlobalMap();
  }

  for (int cy = 0; cy < (int)costmap.info.height; cy++) {
    for (int cx = 0; cx < (int)costmap.info.width; cx++) {
      int8_t cell_val = costmap.data[cy * costmap.info.width + cx];
      if (cell_val <= 0) continue;

      // costmap is robot-local: origin is (-WIDTH*RES/2, -HEIGHT*RES/2) in robot frame
      double local_x = costmap.info.origin.position.x + cx * costmap.info.resolution;
      double local_y = costmap.info.origin.position.y + cy * costmap.info.resolution;

      // transform robot-local → world using robot pose (position + yaw)
      double world_x = robot_x + local_x * cos(robot_yaw) - local_y * sin(robot_yaw);
      double world_y = robot_y + local_x * sin(robot_yaw) + local_y * cos(robot_yaw);

      int gx = (int)((world_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int gy = (int)((world_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

      if (gx < 0 || gx >= GLOBAL_WIDTH || gy < 0 || gy >= GLOBAL_HEIGHT) continue;

      int idx = gy * GLOBAL_WIDTH + gx;
      global_map_.data[idx] = cell_val;
    }
  }
}

}
