#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    //The main function the Node will call to get driving instructions
    geometry_msgs::msg::Twist computeCommand(
    const nav_msgs::msg::Path::SharedPtr& path, 
    const nav_msgs::msg::Odometry::SharedPtr& odom);

  private:
    rclcpp::Logger logger_;
    lookahead_distance_ = 1.0;  // Lookahead distance
    goal_tolerance_ = 0.1;     // Distance to consider the goal reached
    linear_speed_ = 0.5;       // Constant forward speed

    // Helper math functions
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path::SharedPtr& path, 
        const geometry_msgs::msg::Point& robot_pos);
    //value computes distance from the A* algorithm
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    //Takes in Yaw value
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);
};

} 

#endif 
