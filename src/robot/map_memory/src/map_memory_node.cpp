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
  setupEmptyGlobalMap();
}

// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::setupEmptyGlobalMap()
{
  global_map_.info.resolution = 0.1;
  global_map_.info.origin.position.x = -15.0;
  global_map_.info.origin.position.y = -15.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.info.width = GLOBALMAPSIZE;
  global_map_.info.height = GLOBALMAPSIZE;
  global_map_.data.resize(GLOBALMAPSIZE * GLOBALMAPSIZE, 0);
  has_global_map_ = true;
}

// Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  yaw = qToYaw(msg->pose.pose.orientation);
  // Compute distance traveled
  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  if (distance >= distance_threshold)
  {
    last_x = x;
    last_y = y;
    should_update_map_ = true;
  }
}

double MapMemoryNode::qToYaw(const geometry_msgs::msg::Quaternion& q) {
  double sin = 2.0 * (q.w * q.z + q.x * q.y);
  double cos = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(sin, cos);
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
    costmap_updated_ = false;
  }
}

// // Integrate the latest costmap into the global map
// void MapMemoryNode::integrateCostmap()
// {
//   // Transform and merge the latest costmap into the global map
//   // (Implementation would handle grid alignment and merging logic)
// }

void MapMemoryNode::integrateCostmap() {
  int costmap_w = latest_costmap_.info.width;
  int costmap_h = latest_costmap_.info.height;
  double resolution = latest_costmap_.info.resolution;

  // Initialize a temporary 2D grid to store merged results
  std::vector<std::vector<int>> temp_grid(GLOBALMAPSIZE, std::vector<int>(GLOBALMAPSIZE, -1));

  for (int i = 0; i < costmap_h; ++i) {
    for (int j = 0; j < costmap_w; ++j) {
      int8_t costmap_val = latest_costmap_.data[i * costmap_w + j];
      if (costmap_val == 0) continue;

      double rel_x = j * resolution + latest_costmap_.info.origin.position.x;
      double rel_y = i * resolution + latest_costmap_.info.origin.position.y;

      // Rotate relative to robot yaw
      double rotated_x = rel_x * std::cos(yaw) - rel_y * std::sin(yaw);
      double rotated_y = rel_x * std::sin(yaw) + rel_y * std::cos(yaw);

      // Translate to global
      double global_x = rotated_x + last_x;
      double global_y = rotated_y + last_y;

      // Convert to grid
      int gx = (int)((global_x - global_map_.info.origin.position.x) / resolution);
      int gy = (int)((global_y - global_map_.info.origin.position.y) / resolution);

      if (gx < 0 || gx >= GLOBALMAPSIZE || gy < 0 || gy >= GLOBALMAPSIZE) continue;

      // Blend with old value
      int prev = temp_grid[gy][gx];
      int blended = (prev < 0) ? costmap_val : (int)(costmap_val * 0.8 + prev * 0.2);
      temp_grid[gy][gx] = blended;
    }
  }

  global_map_.header.frame_id = "sim_world";
  global_map_.header.stamp = this->now();

  // Flatten 2D to 1D
  for (int i = 0; i < GLOBALMAPSIZE; ++i) {
    for (int j = 0; j < GLOBALMAPSIZE; ++j) {
      int idx = i * GLOBALMAPSIZE + j;
      if (temp_grid[i][j] > -1 && global_map_.data[idx] < temp_grid[i][j]) {
        global_map_.data[idx] = temp_grid[i][j];
      }
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
