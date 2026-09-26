#include "map_memory_node.hpp"

//Constructor, starts all variables and sets up the ROS stuff
MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  map_memory_(robot::MapMemoryCore(this->get_logger())),
  last_x_(0.0), //robots x position at last update
  last_y_(0.0), //robots y position at last update
  current_x_(0.0), //robot's current x position
  current_y_(0.0), // robot's current y position
  distance_threshold_(0.3), //how far robot must move before updating map (I changed it to 0.3 for debugging purposes)
  costmap_updated_(false), //has a new costmap arrived since last update
  should_update_map_(false) //has the robot moved far enough to update
{
  //Subscribes to the costmap
  //Listens for new pieces of information about the map and triggers costmapcallback when one arrives
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  //Subscribe to the Odometry (GPS kinda thing)
  //Listens for position updates and triggers odomCallback when they arrive     
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  //Publisher for the global map
  //Broadcasts the stiched-together masterpiece out to the "/map" topic
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  //This is the timer to stop millions of updates happening every second, instead an update happens every second instead.
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapMemoryNode::updateMap, this));


  // Publish empty map immediately so planner has something to start with
  global_map_.info.resolution = 0.1;
  global_map_.info.width      = 500;
  global_map_.info.height     = 500;
  global_map_.info.origin.position.x = -(500 * 0.1) / 2.0;
  global_map_.info.origin.position.y = -(500 * 0.1) / 2.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.header.frame_id = "sim_world";  // ← critical
  global_map_.data.assign(500 * 500, -1);
  map_pub_->publish(global_map_);
}

//Trried every time the costmap node publishes a new part of the map
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

//trriggered when the robots tracker shows a new position
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  current_x_ = x;
  current_y_ = y;

  //Compute distance traveled using pythagorean theorm, showed in the WAT notes in the steps)
  double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
  //if the robot has moved than 1.5 meters
  if (distance >= distance_threshold_) {
      last_x_ = x;
      last_y_ = y;
      should_update_map_ = true; // Flip the flag telling the timer it's time to stitch a new map
  }  
}

//Triggered once every second by the timer 
void MapMemoryNode::updateMap() {
    //Only do the heavy lifting of stitching maps if we actually have new data and we moved far enough
  if (should_update_map_ && costmap_updated_) {
      map_memory_.integrateCostmap(global_map_, latest_costmap_, current_x_, current_y_);
      global_map_.header.stamp = this->get_clock()->now();  
      global_map_.header.frame_id = "sim_world";               
      map_pub_->publish(global_map_);
      should_update_map_ = false;
  }
}

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
