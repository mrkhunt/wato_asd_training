#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger()))
{

  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg)
      { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]()
      { controlLoop(); });
}

void ControlNode::controlLoop()
{
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty())
    return;

  auto robot = robot_odom_->pose.pose.position;
  auto goal = current_path_->poses.back().pose.position;

  if (computeDistance(robot, goal) < goal_tolerance_)
  {
    RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping robot.");
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return;
  }

  auto lookahead = findLookaheadPoint();
  if (!lookahead)
  {
    RCLCPP_WARN(this->get_logger(), "No valid lookahead point found.");
    return;
  }

  auto cmd = computeVelocity(*lookahead);
  cmd_vel_pub_->publish(cmd);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint()
{
  auto robot = robot_odom_->pose.pose.position;

  for (const auto &pose : current_path_->poses)
  {
    if (computeDistance(robot, pose.pose.position) >= lookahead_distance_)
    {
      return pose;
    }
  }

  // If no valid point found, return last
  if (computeDistance(robot, current_path_->poses.back().pose.position) >= goal_tolerance_)
  {
    return current_path_->poses.back();
  }

  return std::nullopt;
}

geometry_msgs::msg::Point ControlNode::transformToRobotFrame(
    const geometry_msgs::msg::Point &global_point,
    const geometry_msgs::msg::Pose &robot_pose)
{
  double yaw = extractYaw(robot_pose.orientation);
  double dx = global_point.x - robot_pose.position.x;
  double dy = global_point.y - robot_pose.position.y;

  geometry_msgs::msg::Point robot_frame_point;
  robot_frame_point.x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  robot_frame_point.y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  robot_frame_point.z = 0.0;
  return robot_frame_point;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_speed_;

  geometry_msgs::msg::Point relative = transformToRobotFrame(target.pose.position, robot_odom_->pose.pose);

  double x_l = relative.x;
  double y_l = relative.y;

  if (x_l == 0 && y_l == 0)
    return cmd;

  double alpha = std::atan2(y_l, x_l);
  double L = std::sqrt(x_l * x_l + y_l * y_l);
  double curvature = 2 * std::sin(alpha) / L;
  cmd.angular.z = linear_speed_ * curvature;

  return cmd;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
