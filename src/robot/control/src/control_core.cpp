#include "control_core.hpp"
#include <cmath>

namespace robot {

// Constructor — no need to set parameters again, already set in header
ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

geometry_msgs::msg::Twist ControlCore::computeCommand(
    const nav_msgs::msg::Path::SharedPtr& path,
    const nav_msgs::msg::Odometry::SharedPtr& odom) {

  geometry_msgs::msg::Twist cmd_vel;

  // Extract robot position and heading
  auto robot_pos  = odom->pose.pose.position;
  double robot_yaw = extractYaw(odom->pose.pose.orientation);

  // If within tolerance of final goal, stop
  if (!path->poses.empty()) {
    auto final_goal = path->poses.back().pose.position;
    if (computeDistance(robot_pos, final_goal) <= goal_tolerance_) {
      RCLCPP_INFO(logger_, "Goal reached! Stopping.");
      cmd_vel.linear.x  = 0.0;
      cmd_vel.angular.z = 0.0;
      return cmd_vel;
    }
  }

  // Find the lookahead point on the path
  auto lookahead_point = findLookaheadPoint(path, robot_pos);
  if (!lookahead_point) {
    return cmd_vel;  // no valid point found, send zero velocity
  }

  // Vector from robot to lookahead point
  double dx = lookahead_point->pose.position.x - robot_pos.x;
  double dy = lookahead_point->pose.position.y - robot_pos.y;

  // Angle to target in world frame
  double angle_to_target = std::atan2(dy, dx);

  // Steering error — difference between where we face and where we want to go
  double alpha = angle_to_target - robot_yaw;

  // Normalize to [-PI, PI] so robot always takes shortest turn
  while (alpha >  M_PI) alpha -= 2.0 * M_PI;
  while (alpha < -M_PI) alpha += 2.0 * M_PI;

  // Pure Pursuit steering formula
  cmd_vel.linear.x  = linear_speed_;
  cmd_vel.angular.z = (2.0 * linear_speed_ * std::sin(alpha)) / lookahead_distance_;

  return cmd_vel;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path::SharedPtr& path,
    const geometry_msgs::msg::Point& robot_pos) {

  // Find first waypoint further than lookahead distance
  for (const auto& waypoint : path->poses) {
    if (computeDistance(robot_pos, waypoint.pose.position) >= lookahead_distance_) {
      return waypoint;
    }
  }

  // If whole path is closer than lookahead, aim for the end
  if (!path->poses.empty()) return path->poses.back();
  return std::nullopt;
}

double ControlCore::computeDistance(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b) {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
  // Convert quaternion to yaw angle (2D heading)
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}
