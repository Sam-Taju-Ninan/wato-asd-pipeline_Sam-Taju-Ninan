#include "control_core.hpp"
#include <cmath>

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {
    //Initialize the parameters
}

geometry_msgs::msg::Twist ControlCore::computeCommand(
    const nav_msgs::msg::Path::SharedPtr& path, 
    const nav_msgs::msg::Odometry::SharedPtr& odom) {
    
    //Implement logic to compute velocity commands
    geometry_msgs::msg::Twist cmd_vel;
    
    //Extract the robot's current X/Y position and yaw
    auto robot_pos = odom->pose.pose.position;
    double robot_yaw = extractYaw(odom->pose.pose.orientation);

    //If we are within the place of the final point, stop.
    if (!path->poses.empty()) {
        auto final_goal = path->poses.back().pose.position;
        if (computeDistance(robot_pos, final_goal) <= goal_tolerance_) {
            RCLCPP_INFO(logger_, "Goal reached! Stopping.");
            return cmd_vel; // Returns default 0.0 velocity
        }
    }

    //Find the lookahead point
    auto lookahead_point = findLookaheadPoint(path, robot_pos);
    if (!lookahead_point) {
        return cmd_vel; //No valid lookahead point found, just a condition 
    }

    //Find the horizontal and vertical distance
    double dx = lookahead_point->pose.position.x - robot_pos.x;
    double dy = lookahead_point->pose.position.y - robot_pos.y;
    
    //Find the raw angle to the target point
    double angle_to_target = std::atan2(dy, dx);
    
    //Find the difference between where we are facing and where the target is 
    double alpha = angle_to_target - robot_yaw;
    
    //Normalize Alpha to stay between -PI and PI so the robot takes the shortest turn
    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;

    //Apply the steering formula
    cmd_vel.linear.x = linear_speed_; 
    cmd_vel.angular.z = (2.0 * linear_speed_ * std::sin(alpha)) / lookahead_distance_;
    
    return cmd_vel;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path::SharedPtr& path, 
    const geometry_msgs::msg::Point& robot_pos) {
    
    //Loop through the path dots until we find the first one that is further than our lookahead distance
    for (const auto& waypoint : path->poses) {
        if (computeDistance(robot_pos, waypoint.pose.position) >= lookahead_distance_) {
            return waypoint; //Replace with a valid point when implemented
        }
    }
    
    //If the whole path is shorter than our lookahead distance, just aim for the very end
    if (!path->poses.empty()) {
        return path->poses.back();
    }
    return std::nullopt; 
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    //distance calculation between two points
    //Pythagorean theorem
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    //Implement quaternion to yaw conversion
    //Translates 3D spatial rotation into a simple 2D left/right compass angle (yaw ofc)
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}
