#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
 
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop(){
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
      geometry_msgs::msg::Twist stop;
      stop.linear.x = 0.0;
      stop.angular.z = 0.0;
      cmd_vel_pub_->publish(stop);
      return;
  }

  double dist_to_goal = control_.computeDistance(
      robot_odom_->pose.pose.position,
      current_path_->poses.back().pose.position);
  if (dist_to_goal < goal_tolerance_) {
      geometry_msgs::msg::Twist stop;
      stop.linear.x = 0.0;
      stop.angular.z = 0.0;
      cmd_vel_pub_->publish(stop);
      current_path_ = nullptr;
      return;
  }

  auto lookahead_point = control_.findLookaheadPoint(robot_odom_, current_path_, lookahead_distance_);
  if (!lookahead_point) {
      return;
  }

  auto cmd_vel = control_.computeVelocity(*lookahead_point, linear_speed_, robot_odom_, lookahead_distance_);
  cmd_vel_pub_->publish(cmd_vel);
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
