#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void MapMemoryCore::integrateCostmap(nav_msgs::msg::OccupancyGrid& global_map,const nav_msgs::msg::OccupancyGrid& latest_costmap,double robot_x,double robot_y) {
  //Step 1: Init the global map if this is the first update

   if (global_map.data.empty()) {
    global_map.info.resolution = RESOLUTION;
    global_map.info.width      = WIDTH;
    global_map.info.height     = HEIGHT;
    global_map.info.origin.position.x = -(WIDTH  * RESOLUTION) / 2.0;
    global_map.info.origin.position.y = -(HEIGHT * RESOLUTION) / 2.0;
    global_map.info.origin.position.z = 0.0;
    global_map.info.origin.orientation.w = 1.0;
    global_map.header.frame_id = "sim_world";
    global_map.header.stamp = latest_costmap.header.stamp;
    global_map.data.assign(WIDTH * HEIGHT, -1);  

    }

    //Step 2: Loop through every cell in the costmap

    for (int i = 0; i <(int)latest_costmap.data.size(); i++){
      int value = latest_costmap.data[i];

      //Skip unknown cells, keep whatever is in global map
      if (value == -1){
        continue;
      }

      //Step 3: Convert 1D index to 2D costmap coordinates
      int cx = i % latest_costmap.info.width;
      int cy = i / latest_costmap.info.width;

      //Convert to the real world position
      //Costmap is centered on robot, so it adds the robot position
      double x = latest_costmap.info.origin.position.x + cx * latest_costmap.info.resolution + robot_x;
      double y = latest_costmap.info.origin.position.y + cy * latest_costmap.info.resolution + robot_y;

      //This converts the real world position to the global map index 
      //how far from map corner (meters), divide by cell size, get cell index. 
      //NOTE: Static cast is there since a division gives a decimal. Grid indices must be whole numbers, so this convers it to an integer by dropping the decimal part
  
      int gx = static_cast<int>((x - global_map.info.origin.position.x) / RESOLUTION); //x-axis
      int gy = static_cast<int>((y - global_map.info.origin.position.y) / RESOLUTION); //y-axis

      // Bounds check
      if (gx < 0 || gx >= WIDTH || gy < 0 || gy >= HEIGHT) continue;

      // Overwrite global map cell with new value
      global_map.data[gy * WIDTH + gx] = value;

    //



    
  }
}

}