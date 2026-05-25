#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <optional>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(const nav_msgs::msg::Odometry::SharedPtr odom, nav_msgs::msg::Path::SharedPtr path, const double distance);
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target, 
      const double linspeed, const nav_msgs::msg::Odometry::SharedPtr odom, const double distance);
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);
  
  private:
    rclcpp::Logger logger_;
};

} 

#endif 
