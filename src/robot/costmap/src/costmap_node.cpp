#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  // Initialize the constructs and their parameters
  // string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  // Subscriber to /lidar topic
  lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  // Publisher to /costmap topic
  costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage()
{
  // auto message = std_msgs::msg::String();
  // message.data = "Hello, ROS 2!";
  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  // string_pub_->publish(message);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // Step 1: Initialize costmap
  initializeCostmap();

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min)
    {
      // Calculate grid coordinates
      int x_grid = 0;
      int y_grid = 0;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  // Step 3: Inflate obstacles
  inflateObstacles();

  // Step 4: Publish costmap
  publishCostmap(scan);
}

void CostmapNode::initializeCostmap()
{
  occupancy_grid = std::vector<std::vector<int8_t>>(GRIDSIZE, std::vector<int8_t>(GRIDSIZE, 0)); // Initialize all cells to a default value (e.g., 0 for free space).
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid)
{
  // Convert polar coordinates to grid coordinates
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);
  x_grid = static_cast<int>((x - origin_x_) / resolution_);
  y_grid = static_cast<int>((y - origin_y_) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid)
{
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_)
  {
    occupancy_grid[y_grid][x_grid] = max_cost_; // Mark as occupied
  }
}

void CostmapNode::inflateObstacles()
{
  // Inflate obstacles based on inflation radius
  for (int y = 0; y < GRIDSIZE; ++y)
  {
    for (int x = 0; x < GRIDSIZE; ++x)
    {
      if (occupancy_grid[y][x] == max_cost_)
      {
        for (int dy = -inflation_radius_; dy <= inflation_radius_; ++dy)
        {
          for (int dx = -inflation_radius_; dx <= inflation_radius_; ++dx)
          {
            if (std::sqrt(dx * dx + dy * dy) <= inflation_radius_)
            {
              int new_x = x + dx;
              int new_y = y + dy;
              if (new_x >= 0 && new_x < GRIDSIZE && new_y >= 0 && new_y < GRIDSIZE)
              {
                int8_t new_cost = static_cast<int8_t>(max_cost_ * (1 - (sqrt(dx * dx + dy * dy) / inflation_radius_)));
                occupancy_grid[new_y][new_x] = std::max(occupancy_grid[new_y][new_x], new_cost);
              }
            }
          }
        }
      }
    }
  }
}

void CostmapNode::publishCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // header
  costmap_msg.header = scan->header;

  // info
  costmap_msg.info.resolution = resolution_;
  costmap_msg.info.width = GRIDSIZE;
  costmap_msg.info.height = GRIDSIZE;
  costmap_msg.info.origin.position.x = origin_x_;
  costmap_msg.info.origin.position.y = origin_y_;
  costmap_msg.info.origin.orientation.w = orientation_w_;

  // Flatten the 2D costmap array into a 1D array
  costmap_msg.data.resize(width_ * height_, -1);
  for (int y = 0; y < GRIDSIZE; ++y)
  {
    for (int x = 0; x < GRIDSIZE; ++x)
    {
      int index = y * GRIDSIZE + x;
      costmap_msg.data[index] = std::max(occupancy_grid[y][x], static_cast<int8_t>(0));
    }
  }

  // publish message
  costmap_publisher_->publish(costmap_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}