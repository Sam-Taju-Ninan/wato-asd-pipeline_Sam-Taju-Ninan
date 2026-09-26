#include <chrono>
#include <memory>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  //Subscriber which listens for lidar data
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
  );

  //Publisher which sends costmap to map memory node
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  //Step 1 it creates a fresh blank grid
  auto grid = costmap_.initializeCostmap();

  // Step 2 it loops through every lidar beam
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    //Filter out any bad readings
    if (range > scan->range_min && range < scan->range_max) {
      int x_grid, y_grid;
      //Convert beam to grid cell
      costmap_.convertToGrid(range, angle, x_grid, y_grid, grid);
      //Mark that cell as obstacle
      costmap_.markObstacle(grid, x_grid, y_grid);
    }
  }

  //Step 3 it spread costs outward from obstacles
  costmap_.inflateObstacles(grid);

  //Step 4 it gets published
  grid.header.stamp    = this->get_clock()->now();
  grid.header.frame_id = "sim_world";
  costmap_pub_->publish(grid);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}