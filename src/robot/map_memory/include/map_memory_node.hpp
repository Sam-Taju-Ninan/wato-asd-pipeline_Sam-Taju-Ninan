#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp" // for costmap and global map messages
#include "nav_msgs/msg/odometry.hpp" //for the robot position messages 
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    //Core logic object, which handles the actual map merging math
    robot::MapMemoryCore map_memory_;

    //The Subscribers, publisher, timer that are being used 
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    //The data that is collected from the costmap core. and now being used for the global map
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    double last_x_, last_y_;
    double current_x_, current_y_;
    const double distance_threshold_;

    //Flags
    bool costmap_updated_;
    bool should_update_map_;

    //Callbacks to certain functions 
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
};

#endif 
