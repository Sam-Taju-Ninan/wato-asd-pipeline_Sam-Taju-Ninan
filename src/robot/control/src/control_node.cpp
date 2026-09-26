#include "control_node.hpp"

ControlNode::ControlNode() 
: Node("control"), 
  control_(robot::ControlCore(this->get_logger())) {
    
    //Subscribers and Publishers now being used for 

    //path sub is like the map reader
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { 
            current_path_ = msg; 
            control_.current_path_index_ = 0;
        });

    //odom is the GPS receiver kinda
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { current_odom_ = msg; });

    //CMD is the steering wheel 
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
    //Skip control if no path or odometry data is available
    if (!current_path_ || !current_odom_) {
        return;
    }

    //Pass the data to our core math file to compute velocity command
    auto cmd_vel = control_.computeCommand(current_path_, current_odom_);
    
    //Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel);
}

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
