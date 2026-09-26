#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
//Brings in the OccupancyGrid message type
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

//This is the core logic class  
class MapMemoryCore {
  public:
  //constructor that takes a logger so it can print messages to the terminal. 
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    //The main function that the map_memory will use to make the global map.
    void integrateCostmap(
    nav_msgs::msg::OccupancyGrid& global_map,
    const nav_msgs::msg::OccupancyGrid& latest_costmap,
    double robot_x,
    double robot_y);

  private:
    rclcpp::Logger logger_;

    //Constants that will never change. 
    //Map memory, that the robot will always remember, but not more than that. 
    static constexpr double RESOLUTION = 0.1;
    static constexpr int    WIDTH      = 500;  // 50m
    static constexpr int    HEIGHT     = 500;  // 50m
};

}  

#endif  
