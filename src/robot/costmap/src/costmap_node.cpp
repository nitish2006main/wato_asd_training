#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
 
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
  costmap_.initializeCostmap();
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range > scan->range_min && range < scan->range_max) {
      double x = range * cos(angle);
      double y = range * sin(angle);
      int x_grid = (int)(x / robot::CostmapCore::RESOLUTION) + robot::CostmapCore::WIDTH / 2;
      int y_grid = (int)(y / robot::CostmapCore::RESOLUTION) + robot::CostmapCore::HEIGHT / 2;
      costmap_.markObstacle(x_grid, y_grid);
    }
  }
  costmap_.inflateObstacles();
  nav_msgs::msg::OccupancyGrid costmap_msg = nav_msgs::msg::OccupancyGrid();
  costmap_msg.header.stamp = this->get_clock()->now();
  costmap_msg.header.frame_id = "robot";
  costmap_msg.info.resolution = robot::CostmapCore::RESOLUTION;
  costmap_msg.info.width = robot::CostmapCore::WIDTH;
  costmap_msg.info.height = robot::CostmapCore::HEIGHT;
  costmap_msg.info.origin.position.x = -(robot::CostmapCore::WIDTH * robot::CostmapCore::RESOLUTION) / 2;
  costmap_msg.info.origin.position.y = -(robot::CostmapCore::HEIGHT * robot::CostmapCore::RESOLUTION) / 2;
  for (int y = 0; y < robot::CostmapCore::HEIGHT; y++){
    for (int x = 0; x < robot::CostmapCore::WIDTH; x++){
      costmap_msg.data.push_back(costmap_.getCost(x, y));
    }
  }
  occupancy_pub_->publish(costmap_msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}