#ifndef WAYPOINT_PUBLISHER_CORE_HPP_
#define WAYPOINT_PUBLISHER_CORE_HPP_

#include <cmath>
#include <stdexcept>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>

#define FOLLOW_WAYPOITNS_MODE 1
#define THROUGH_POSES_MODE 0

#define SEND_WAYPOINTS 0
#define SEND_WAYPOINTS_CHECK 1
#define WAITING_GOAL 2
#define WAITING_BUTTON 3
#define FINISH_SENDING 4

using namespace std::chrono_literals;

typedef struct{
  geometry_msgs::msg::Pose poses;
  bool will_stop;
}waypoint_info;

class WayPointPublisher : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
  WayPointPublisher();

private:
  std::vector<std::string> getCSVLine(std::string& input, char delimiter);
  void declareParams();
  void getParams();
  bool checkParameters(const std::vector<bool>& list);
  void ReadWaypointsFromCSV(std::string& csv_file, std::vector<waypoint_info>& res_waypoints);
  void PublishWaypointMarkers(const std::vector<waypoint_info> waypoints, size_t start_index);
  void NavThroughPosesFeedbackCallback(const GoalHandleNavigateNavigateThroughPoses::SharedPtr,const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
  void NavThroughPosesResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result);
  void NavThroughPosesGoalResponseCallback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>> future);
  void JoyCallback(const sensor_msgs::msg::Joy &joy_msg);
  void SendWaypointsTimerCallback();
  size_t SendWaypointsOnce(size_t sending_index);

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_text_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Clock ros_clock(RCL_ROS_TIME);
  int id_;
  std::string csv_file_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_goal_handle_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_goal_handle_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions send_goal_options_;
  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>>> future_goal_handle_;

  std::vector<waypoint_info> waypoints_;
  size_t start_index_;

  int follow_type_;  // navigate_through_pose:0, follow_waypoints:1
  int start_index_int_;
  bool is_action_server_ready_;
  bool is_goal_achieved_;
  bool is_aborted_;
  bool is_standby_;
  bool is_goal_accepted_;
  float waypoint_marker_scale_;
  float waypoint_marker_color_r_;
  float waypoint_marker_color_g_;
  float waypoint_marker_color_b_;
  float waypoint_marker_color_a_;
  float waypoint_stop_marker_color_r_;
  float waypoint_stop_marker_color_g_;
  float waypoint_stop_marker_color_b_;
  float waypoint_stop_marker_color_a_;

  float waypoint_text_marker_scale_;
  float waypoint_text_marker_color_r_;
  float waypoint_text_marker_color_g_;
  float waypoint_text_marker_color_b_;
  float waypoint_text_marker_color_a_;
  int16_t number_of_poses_remaining_;
};

#endif