#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{

  // subscribe to /costmap
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  // subscribe to /odom/filtered
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // publish to /map
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // timer
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  // === Initialize empty map ===
  global_map_.info.resolution = 0.1;
  global_map_.info.origin.position.x = -20.0;
  global_map_.info.origin.position.y = -20.0;
  global_map_.info.origin.orientation.w = orientation_w_;
  global_map_.info.width = GRIDSIZE;
  global_map_.info.height = GRIDSIZE;
  global_map_.data = std::vector<int8_t>(GRIDSIZE * GRIDSIZE, 0);
  has_global_map_ = true;
  costmap_processing_ = false;
}

// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

// Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // update the values
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  yaw = qToYaw(msg->pose.pose.orientation);

  // Compute distance traveled
  double distance = std::hypot(x - last_x, y - last_y);
  if (distance >= distance_threshold)
  {
    last_x = x;
    last_y = y;
    should_update_map_ = true;
    costmap_processing_ = false;
  }
}

double MapMemoryNode::qToYaw(const geometry_msgs::msg::Quaternion &q)
{
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return yaw;
}

// Timer-based map update
void MapMemoryNode::updateMap()
{
  if (should_update_map_ && costmap_updated_)
  {
    integrateCostmap();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
  }
}

void MapMemoryNode::integrateCostmap()
{
  double oxg = x + latest_costmap_.info.origin.position.x;
  double oyg = y + latest_costmap_.info.origin.position.y;

  double cx = GRIDSIZE / 2.0;
  double cy = GRIDSIZE / 2.0;
  double res = latest_costmap_.info.resolution;

  for (unsigned int iy = 0; iy < latest_costmap_.info.height; ++iy)
  {
    for (unsigned int ix = 0; ix < latest_costmap_.info.width; ++ix)
    {
      double dx = (ix - cx) * res;
      double dy = (iy - cy) * res;

      double rot_x = cos(yaw) * dx - sin(yaw) * dy;
      double rot_y = sin(yaw) * dx + cos(yaw) * dy;

      double global_x_m = rot_x + oxg + cx * res;
      double global_y_m = rot_y + oyg + cy * res;

      int gx = static_cast<int>((global_x_m - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int gy = static_cast<int>((global_y_m - global_map_.info.origin.position.y) / global_map_.info.resolution);

      if (gx >= 0 && gx < static_cast<int>(global_map_.info.width) &&
          gy >= 0 && gy < static_cast<int>(global_map_.info.height))
      {
        int cost_idx = iy * latest_costmap_.info.width + ix;
        if (latest_costmap_.data[cost_idx] > 0)
        {
          global_map_.data[gy * global_map_.info.width + gx] =
              latest_costmap_.data[cost_idx];
        }
      }
    }
  }

  global_map_.header = latest_costmap_.header;
  global_map_.header.frame_id = "sim_world";
  costmap_updated_ = false;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
