#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

const int GRIDSIZE = 400;

class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode();

  // Place callback function here
  void publishMessage();

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void initializeCostmap();
  void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
  void markObstacle(int x_grid, int y_grid);
  void inflateObstacles();
  void publishCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan);

private:
  // Place these constructs here
  robot::CostmapCore costmap_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;

  std::vector<std::vector<int8_t>> occupancy_grid; // occupancy grid
  double resolution_ = 0.1;                        // 0.1 meters per cell
  int width_ = GRIDSIZE;
  int height_ = GRIDSIZE;
  double inflation_radius_ = 10; // 1 meter inflation radius
  int max_cost_ = 100;           // Max cost for occupied cells
  double origin_x_ = -20.0;
  double origin_y_ = -20.0;
  double orientation_w_ = 1.0;

  nav_msgs::msg::OccupancyGrid costmap_msg;
};

#endif