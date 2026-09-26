#include "control_core.hpp"
#include <cmath>

namespace robot {

// Constructor — no need to set parameters again, already set in header
ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {
    linear_speed_ = 1.5;
    lookahead_distance_ = 1.0;
    goal_tolerance_ = 0.5;
  }

geometry_msgs::msg::Twist ControlCore::computeCommand(
    const nav_msgs::msg::Path::SharedPtr& path,
    const nav_msgs::msg::Odometry::SharedPtr& odom) {

  geometry_msgs::msg::Twist cmd_vel;

  // 1. SAFETY STOP:
    if (path->poses.empty() || current_path_index_ >= path->poses.size()) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return cmd_vel; // Instantly return the zeroed speeds
    }

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

  // TURN-IN-PLACE THRESHOLD
  double k_theta = 1.5; // Steering sensitivity

  if (std::abs(alpha) > 0.6) {
      // If the turn is sharp (> ~35 degrees), stop forward movement and ONLY spin
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = k_theta * alpha;
  } else {
      // If the robot is mostly facing the right way, drive forward using Pure Pursuit
      cmd_vel.linear.x = linear_speed_;
      cmd_vel.angular.z = (2.0 * linear_speed_ * std::sin(alpha)) / lookahead_distance_;
  }

  // CLAMP MAXIMUM SPIN SPEED to prevent violent shaking
  double max_spin = 1.2;
  if (cmd_vel.angular.z > max_spin) cmd_vel.angular.z = max_spin;
  if (cmd_vel.angular.z < -max_spin) cmd_vel.angular.z = -max_spin;

  return cmd_vel;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path::SharedPtr& path, 
    const geometry_msgs::msg::Point& robot_pos) {
    
    if (path->poses.empty()) {
        return std::nullopt;
    }

    // 1. Find the index of the closest point on the path to the robot
    size_t closest_index = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path->poses.size(); ++i) {
        double dist = computeDistance(robot_pos, path->poses[i].pose.position);
        if (dist < min_dist) {
            min_dist = dist;
            closest_index = i;
        }
    }

    // 2. Scan forward from the closest point to find the first waypoint >= lookahead_distance_
    for (size_t i = closest_index; i < path->poses.size(); ++i) {
        if (computeDistance(robot_pos, path->poses[i].pose.position) >= lookahead_distance_) {
            return path->poses[i];
        }
    }
    
    // 3. If everything ahead is within the lookahead distance, aim for the end of the path
    return path->poses.back();
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
