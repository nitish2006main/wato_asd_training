#include "control_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Odometry::SharedPtr odom, nav_msgs::msg::Path::SharedPtr path, const double distance)
  {
    double robot_to_lookahead_dist = 0.0;
    for (size_t i = 0; i < path->poses.size(); i++)
    {
      robot_to_lookahead_dist = computeDistance(odom->pose.pose.position, path->poses[i].pose.position);
      if (robot_to_lookahead_dist >= distance)
      {
        return path->poses[i];
      }
    }
    if (path->poses.empty()) return std::nullopt;
    return path->poses.back();
  }
geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped &target, 
  const double linspeed, const nav_msgs::msg::Odometry::SharedPtr odom, const double distance)
  {
    double yaw = extractYaw(odom->pose.pose.orientation);
    double angle_to_target = atan2(target.pose.position.y - odom->pose.pose.position.y, 
      target.pose.position.x - odom->pose.pose.position.x);
    double steering_angle = angle_to_target - yaw;
    double linear_velocity = linspeed;
    double actual_distance = std::max(computeDistance(odom->pose.pose.position, target.pose.position), 1e-3);
    double angular_velocity = 2 * linear_velocity * sin(steering_angle) / std::min(distance, actual_distance);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angular_velocity;
    return cmd_vel;
  }

double ControlCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b){
  double distance = sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
  return distance;
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat){
  double yaw = atan2(2 * (quat.w*quat.z + quat.x*quat.y), 1 - 2 * (quat.y*quat.y + quat.z*quat.z));
  return yaw;
}

}


