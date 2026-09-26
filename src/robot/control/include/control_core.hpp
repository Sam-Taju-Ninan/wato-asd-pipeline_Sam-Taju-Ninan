#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>

namespace robot {

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);

    // Main function the node calls to get driving instructions
    geometry_msgs::msg::Twist computeCommand(
        const nav_msgs::msg::Path::SharedPtr& path,
        const nav_msgs::msg::Odometry::SharedPtr& odom);

  private:
    rclcpp::Logger logger_;

    // Tunable parameters
    double lookahead_distance_ = 2.0;  // how far ahead to aim
    double goal_tolerance_     = 0.5;  // how close = "arrived"
    double linear_speed_       = 0.3;  // constant forward speed

    // Helper functions
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path::SharedPtr& path,
        const geometry_msgs::msg::Point& robot_pos);

    double computeDistance(
        const geometry_msgs::msg::Point& a,
        const geometry_msgs::msg::Point& b);

    double extractYaw(const geometry_msgs::msg::Quaternion& quat);
};

}

#endif
