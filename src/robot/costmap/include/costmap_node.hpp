#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp" 
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    robot::CostmapCore costmap_;

    //Subscriber, it listens for lidar data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    // Publisher, it sends costmap to map memory node
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // Callback calls automatically every time a lidar scan arrives
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};

#endif