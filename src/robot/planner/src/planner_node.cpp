#include "planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
 
  // Timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (!goal_received_ || current_map_.data.empty()) {
      return;
    }
    nav_msgs::msg::Path path = planner_.planPath(current_map_, robot_pose_, goal_);
    path_pub_->publish(path);
  }
}
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "Goal received: x=%f, y=%f", goal_.point.x, goal_.point.y);
  if (current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: map not yet available");
    return;
  }
  nav_msgs::msg::Path path = planner_.planPath(current_map_, robot_pose_, goal_);
  RCLCPP_INFO(this->get_logger(), "Path has %zu poses", path.poses.size());
  path_pub_->publish(path);
}
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}
void PlannerNode::timerCallback(){
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
      goal_received_ = false;  // add this
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      if (!goal_received_ || current_map_.data.empty()) {
        return;
      }
      nav_msgs::msg::Path path = planner_.planPath(current_map_, robot_pose_, goal_);
      path_pub_->publish(path);
    }
  }
}
bool PlannerNode::goalReached(){
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
