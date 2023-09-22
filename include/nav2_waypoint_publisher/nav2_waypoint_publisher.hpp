#ifndef WAYPOINT_PUBLISHER_CORE_HPP_
#define WAYPOINT_PUBLISHER_CORE_HPP_

#include <cmath>
#include <stdexcept>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class WayPointPublisher : public rclcpp::Node
{
public:
  WayPointPublisher();

private:
  std::vector<std::string> getCSVLine(std::string& input, char delimiter);
  void publishWaypointsFromCSV(std::string csv_file);
  void declareParams();
  void getParams();
  bool checkParameters(const std::vector<bool>& list);

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_text_pub_;
  // rclcpp::Clock ros_clock(RCL_ROS_TIME);
  int id_;
  std::string csv_file_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_goal_handle_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_goal_handle_;
  int follow_type_;  // navigate_through_pose:0, follow_waypoints:1
  bool is_action_server_ready_;
  float waypoint_marker_scale_;
  float waypoint_marker_color_r_;
  float waypoint_marker_color_g_;
  float waypoint_marker_color_b_;
  float waypoint_marker_color_a_;

  float waypoint_text_marker_scale_;
  float waypoint_text_marker_color_r_;
  float waypoint_text_marker_color_g_;
  float waypoint_text_marker_color_b_;
  float waypoint_text_marker_color_a_;
};

#endif