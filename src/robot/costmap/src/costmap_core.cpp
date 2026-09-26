#include "costmap_core.hpp"
#include <cmath>
#include <vector>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

//this function initializes the cost map, and creates a blank grid 
nav_msgs::msg::OccupancyGrid CostmapCore::initializeCostmap() {
    nav_msgs::msg::OccupancyGrid grid;

    //sets the data
    grid.info.resolution = RESOLUTION;
    grid.info.width      = WIDTH;
    grid.info.height     = HEIGHT; 

    //center the grid on the robot, by setting the origin
    grid.info.origin.position.x = -(WIDTH  * RESOLUTION) / 2.0;
    grid.info.origin.position.y = -(HEIGHT * RESOLUTION) / 2.0;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    //every cell should be defaulted to zero, the inflation values should only be added when the robot finds an obstacle.

    grid.data.assign(WIDTH * HEIGHT, 0);

    return grid;
}

//This functionConvert the range and angles of the beam to the grid information
void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid,
                                 const nav_msgs::msg::OccupancyGrid& grid) {
    //use range and angle to find x and y
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    //Convert real world position to grid index
    //Subtract origin to get position relative to grid corner
    x_grid = static_cast<int>((x - grid.info.origin.position.x) / RESOLUTION);
    y_grid = static_cast<int>((y - grid.info.origin.position.y) / RESOLUTION);
}

//This function Marks cells as obstacles
void CostmapCore::markObstacle(nav_msgs::msg::OccupancyGrid& grid, int x_grid, int y_grid) {
    //It make sure the cell is inside the grid
    if (x_grid < 0 || x_grid >= WIDTH || y_grid < 0 || y_grid >= HEIGHT) return;
    //Convert 2D index to 1D index and set to max cost if it is an obstacle.
    grid.data[y_grid * WIDTH + x_grid] = MAX_COST;
}
//This function  Inflate obstacles outward
void CostmapCore::inflateObstacles(nav_msgs::msg::OccupancyGrid& grid) {
    int inflate_cells = static_cast<int>(INFLATION_RADIUS / RESOLUTION);

    std::vector<int8_t> inflated = grid.data;

    //Visits every single cell in the grid 
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            //If the cell is not an obstacle, skip it. We only want to inflate actual obstacle cells
            if (grid.data[y * WIDTH + x] != MAX_COST) continue;

            //Check every cell within inflation radius 
            for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
                for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {

                    //Bounds check
                    int nx = x + dx;
                    int ny = y + dy;

                    //Bounds check, skip if outside grid

                    if (nx < 0 || nx >= WIDTH || ny < 0 || ny >= HEIGHT) continue;

                    //Distance in meters
                    double dist = std::sqrt(dx * dx + dy * dy) * RESOLUTION;
                    if (dist > INFLATION_RADIUS) continue;

                    //Cost decreases with distance
                    int cost = static_cast<int>(MAX_COST * (1.0 - dist / INFLATION_RADIUS));

                    //Only overwrite if new cost is higher
                    int idx = ny * WIDTH + nx;
                    if (cost > inflated[idx]) inflated[idx] = cost;
                }
            }
        }
    }

    grid.data = inflated;    
}

}

