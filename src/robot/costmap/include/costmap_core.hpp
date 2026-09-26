#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    //creates blank grid 
    nav_msgs::msg::OccupancyGrid initializeCostmap();
    //Given a distance and angle, tell what grid cell it has landed on.
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid,
                       const nav_msgs::msg::OccupancyGrid& grid);
    //ses obstacles and marks it so that it can avoid it later on
    void markObstacle(nav_msgs::msg::OccupancyGrid& grid, int x_grid, int y_grid);
    //spreads outward from every obstacle.
    void inflateObstacles(nav_msgs::msg::OccupancyGrid& grid);

    //NOTE TO SELF: the & operator in thid case means passing by reference. The function works on the actual variable, not a copy. 

  private:
    rclcpp::Logger logger_;

    //NOTE TO SELF: constexpr means that the value is fixed at compile time and is never changed.

    static constexpr double RESOLUTION     = 0.1; //Each grid cell represents 0.1 meters of real world space. 
    static constexpr int    WIDTH          = 200;   //Grid is 200cells wid so its 20m wide
    static constexpr int    HEIGHT         = 200;   //20m high
    static constexpr double INFLATION_RADIUS = 1.0; //how far to sprea cost around each obstacle
    static constexpr int    MAX_COST       = 100; //highest cost value

};

}  

#endif  