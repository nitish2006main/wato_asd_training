#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    void initializeCostmap();
    void markObstacle(int x, int y);
    void inflateObstacles();
    static constexpr float RESOLUTION = 0.1;
    static const int HEIGHT = 200;
    static const int WIDTH = 200;
    static const int INFLATION_RADIUS = 20;
    static const int MAX_COST = 90;
    int getCost(int x, int y);

  private:
    rclcpp::Logger logger_;
    int costmap_[HEIGHT][WIDTH];

};

}  

#endif  