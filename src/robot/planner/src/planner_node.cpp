#include "planner_node.hpp"

//CONSTRUCTOR: Initializes node name, state, core planner object, and ROS 2 constructs

PlannerNode::PlannerNode() 
: Node("planner"), 
  state_(State::WAITING_FOR_GOAL), //Starts the robot in idle state (waiting for a place to go)
  planner_(robot::PlannerCore(this->get_logger()))  {

  // Subscribers

  //This listens for global Occupancy Grid maps coming from the map memory
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  //This listen sfor a target destination that is clicked on Foxglove     
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  //This listens for the robot's current position that is from the odom    
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher

  //This sends out the calculated path to the control node on the /path topic 
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  //Timer: just calls timercallback every 500ms to check progress or replan
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

//Map callback: This runs automatically whenever a new global map arrives

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  current_map_ = *msg; //Stores latest map

  //If actively navigating toward a goal, recalculate the path using the fresh map
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }

}

//This is the goal callback: Runs automatically when a user clicks a goal point in Foxglove

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

//Now the odom callback: Tracks robot's current position in the world

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback(){
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        state_ = State::WAITING_FOR_GOAL;
    }
  }
}

bool PlannerNode::goalReached(){
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::planPath(){
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  //Passes map, start pose, and goal into PlannerCore A* algorithm
  nav_msgs::msg::Path path = planner_.planPath(current_map_, robot_pose_, goal_);

  path.header.stamp    = this->get_clock()->now();
  path.header.frame_id = "odom";

  //Publish the resulting path to /path topic
  path_pub_->publish(path);
 
   
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;

}
