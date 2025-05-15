#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double g_score;
  double h_score;
  double f_score;
  CellIndex parent;

  AStarNode()
      : index(-1, -1), g_score(0), h_score(0), f_score(0), parent(-1, -1) {}

  AStarNode(CellIndex idx, double g, double h, CellIndex parent_)
      : index(idx), g_score(g), h_score(h), f_score(g + h), parent(parent_) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  bool goalReached();
  void planPath();

  // Utility methods for A* path planning
  CellIndex worldToGrid(double x, double y);
  std::pair<double, double> gridToWorld(const CellIndex &idx);
  double heuristic(const CellIndex &a, const CellIndex &b);
  bool isValid(const CellIndex &idx);
  std::vector<std::pair<int, int>> neighborOffsets();
  void reconstructAndPublishPath(
      const std::unordered_map<CellIndex, AStarNode, CellIndexHash> &closed_set,
      const CellIndex &goal);

private:
  robot::PlannerCore planner_;

  enum class State
  {
    WAITING_FOR_GOAL,
    WAITING_FOR_ROBOT_TO_REACH_GOAL
  };
  State state_;

  // Subscribers and Publisher
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data Storage
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  geometry_msgs::msg::Pose robot_pose_;

  bool goal_received_ = false;
  const double resolution = 0.1;
  const int scale_factor = static_cast<int>(1.0 / resolution);
};

#endif
