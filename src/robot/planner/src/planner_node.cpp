#include "planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), planner_(this->get_logger()), state_(State::WAITING_FOR_GOAL)
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    if (goalReached())
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached()
{
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

// void PlannerNode::planPath()
// {
//   if (!goal_received_ || current_map_.data.empty())
//   {
//     RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
//     return;
//   }

//   // A* Implementation (pseudo-code)
//   nav_msgs::msg::Path path;
//   path.header.stamp = this->get_clock()->now();
//   path.header.frame_id = "map";

//   // Compute path using A* on current_map_
//   // Fill path.poses with the resulting waypoints.

//   path_pub_->publish(path);
// }

CellIndex PlannerNode::worldToGrid(double x, double y)
{
  return CellIndex(
      static_cast<int>((x + 20) * scale_factor),
      static_cast<int>((y + 20) * scale_factor));
}

std::pair<double, double> PlannerNode::gridToWorld(const CellIndex &idx)
{
  return {
      static_cast<double>(idx.x) / scale_factor - 20,
      static_cast<double>(idx.y) / scale_factor - 20};
}

double PlannerNode::heuristic(const CellIndex &a, const CellIndex &b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool PlannerNode::isValid(const CellIndex &idx)
{
  int w = current_map_.info.width;
  int h = current_map_.info.height;
  if (idx.x < 0 || idx.y < 0 || idx.x >= w || idx.y >= h)
    return false;
  int8_t val = current_map_.data[idx.y * w + idx.x];
  return val >= 0 && val < 50; // free cell
}

std::vector<std::pair<int, int>> PlannerNode::neighborOffsets()
{
  return {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
}

void PlannerNode::reconstructAndPublishPath(
    const std::unordered_map<CellIndex, AStarNode, CellIndexHash> &closed_set,
    const CellIndex &goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";

  if (!closed_set.count(goal))
  {
    path_pub_->publish(path);
    return;
  }

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  CellIndex idx = goal;
  while (idx != CellIndex(-1, -1))
  {
    auto [wx, wy] = gridToWorld(idx);
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;
    pose.header.frame_id = "sim_world";
    waypoints.insert(waypoints.begin(), pose);
    idx = closed_set.at(idx).parent;
  }

  path.poses = waypoints;
  path_pub_->publish(path);
}

void PlannerNode::planPath()
{
  if (!goal_received_ || current_map_.data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
  CellIndex goal = worldToGrid(goal_.point.x, goal_.point.y);

  std::unordered_map<CellIndex, AStarNode, CellIndexHash> open_set;
  std::unordered_map<CellIndex, AStarNode, CellIndexHash> closed_set;

  double h_start = heuristic(start, goal);
  open_set[start] = AStarNode(start, 0.0, h_start, CellIndex(-1, -1));

  while (!open_set.empty())
  {
    auto current_it = std::min_element(open_set.begin(), open_set.end(),
                                       [](const auto &a, const auto &b)
                                       { return a.second.f_score < b.second.f_score; });

    AStarNode current = current_it->second;
    open_set.erase(current_it);
    closed_set[current.index] = current;

    if (current.index == goal)
      break;

    for (const auto &[dx, dy] : neighborOffsets())
    {
      CellIndex neighbor(current.index.x + dx, current.index.y + dy);
      if (!isValid(neighbor) || closed_set.count(neighbor))
        continue;

      double move_cost = (dx != 0 && dy != 0) ? 1.41 : 1.0;
      double g = current.g_score + move_cost;
      double h = heuristic(neighbor, goal);

      if (!open_set.count(neighbor) || g < open_set[neighbor].g_score)
      {
        open_set[neighbor] = AStarNode(neighbor, g, h, current.index);
      }
    }
  }

  reconstructAndPublishPath(closed_set, goal);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
