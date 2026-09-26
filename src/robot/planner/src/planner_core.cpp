#include "planner_core.hpp"

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

nav_msgs::msg::Path PlannerCore::planPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::PointStamped& goal_point){

        nav_msgs::msg::Path path;

        //Remember, we have to convert start and end into grid cells
        double res      = map.info.resolution;
        double origin_x = map.info.origin.position.x;
        double origin_y = map.info.origin.position.y;
        int width       = map.info.width;
        int height      = map.info.height;

        int sx = static_cast<int>((start_pose.position.x - origin_x) / res);
        int sy = static_cast<int>((start_pose.position.y - origin_y) / res);
        CellIndex start(sx, sy);

        int gx = static_cast<int>((goal_point.point.x - origin_x) / res);
        int gy = static_cast<int>((goal_point.point.y - origin_y) / res);
        CellIndex goal(gx, gy);

        //Init A* data structures
        //The Open List showing lowest f score 
        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

        //Track the shortest distance from start to any given cell g score
        std::unordered_map<CellIndex, double, CellIndexHash> g_scores;

        //Track where we came from so we can draw the path backward at the end
        std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

        //Start the algorithm

        g_scores[start] = 0.0; //Cost to reach start is 0

        //Calculate the straight-line distance from start to goal for our initial guess (h)
        double initial_h = std::hypot(goal.x - start.x, goal.y - start.y);

        //Add the start node to our queue to kick off the loop
        open_set.emplace(start, initial_h);

        //THE MAIN LOOP (A*)

        while (!open_set.empty()) {
            // Gets best option, Pull the square with the lowest f_score off the top of the queue
            AStarNode current_node = open_set.top();
            CellIndex current = current_node.index;
            open_set.pop();

            //Checks if we are there yet
            if (current == goal) {
                RCLCPP_INFO(logger_, "Goal found! Reconstructing path...");
                // (We will write the path reconstruction block here next)
                return path; 
            }

            //Define the 4 immediate neighbors (Right, Left, Up, Down),
            std::vector<CellIndex> neighbors = {
                CellIndex(current.x + 1, current.y),
                CellIndex(current.x - 1, current.y),
                CellIndex(current.x, current.y + 1),
                CellIndex(current.x, current.y - 1)
            };

            //Now, its calculating the neighbours h + g = f

            for (const CellIndex& neighbor : neighbors){
                //Is the neighbour off the edge?

                if (neighbor.x < 0 || neighbor.x >= width || neighbor.y < 0 || neighbor.y >= height) {
                    continue; // Skip it
                }

                //is it a wall? is it dangerous?
                //Occupancy grids use 0 for free space, 100 for walls, made from costmap.
                //anything over 50 is too dangerous. 
                int flat_index = neighbor.y * width + neighbor.x;

                if (map.data[flat_index] > 50) { 
                    continue; // Skip it
                }

                //Calculate the cost to reach this neighbor. Moving one square costs 1.
                double tentative_g_score = g_scores[current] + 1.0;

                //If we have never visited this neighbor, OR we found a faster route to it:

                if (g_scores.find(neighbor) == g_scores.end() || tentative_g_score < g_scores[neighbor]){

                    //This is the best path to this neighbour so it must be saved

                    came_from[neighbor] = current;
                    g_scores[neighbor] = tentative_g_score;

                    //Calculate h: Straight-line distance from this neighbor to the goal
                    double h_score = std::hypot(goal.x - neighbor.x, goal.y - neighbor.y);
                    
                    //Final Score is the Distance traveled (g) + Estimated distance remaining (h)

                    double f_score = tentative_g_score + h_score;

                    //Push this neighbor onto the priority queue to explore later
                    open_set.emplace(neighbor, f_score);

                }


            }

        }
        return path;

    }

} 
