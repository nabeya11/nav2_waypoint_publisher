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
#include "tsukutsuku2_msgs/msg/waypoints.hpp"
#include "tsukutsuku2_msgs/msg/state_and_feedback.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>

//送信に係る状態量
#define SEND_WAYPOINTS 0
#define SEND_WAYPOINTS_CHECK 1
#define WAITING_GOAL 2
#define STANBY 3
#define CANCEL_GOAL 4
#define WAITING_CANCEL 5

//feedback系の状態量
#define FEEDBACK_STANBY 0
#define FEEDBACK_PROCCESSING 1
#define FEEDBACK_WAITING_GOAL 2
#define FEEDBACK_ABORTED 3
#define FEEDBACK_REJECTED 4
#define FEEDBACK_CANCELING 5

//ロボット全体の状態量
#define STOP 0
#define NORMAL_RUNNING_MODE 1
#define SIGNAL_CHECK_STOP 2
#define UNKOWN_PLACE_RUNNING_MODE 3
#define DELIVERY_RUNNING_MODE 4

using namespace std::chrono_literals;

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
  void NavThroughPosesFeedbackCallback(const GoalHandleNavigateNavigateThroughPoses::SharedPtr,const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
  void NavThroughPosesResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result);
  void NavThroughPosesGoalResponseCallback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>> future);
  void SendWaypointsTimerCallback();
  void SendWaypointsOnce();
  void CancelTask();
  void WaypointsSubCallback(const tsukutsuku2_msgs::msg::Waypoints::SharedPtr waypoints);
  void PublishWaypointMarkers();

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_text_pub_;
  rclcpp::Publisher<tsukutsuku2_msgs::msg::StateAndFeedback>::SharedPtr waypoint_state_pub_;
  rclcpp::Subscription<tsukutsuku2_msgs::msg::Waypoints>::SharedPtr waypoints_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Clock ros_clock(RCL_ROS_TIME);
  int id_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_goal_handle_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions send_goal_options_;
  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>>> future_goal_handle_;

  std::vector<tsukutsuku2_msgs::msg::Waypoint> waypoints_;
  uint8_t latest_waypoint_state_ ;

  bool is_action_server_ready_;
  bool is_goal_achieved_;
  bool is_checked_;
  bool is_aborted_;
  bool is_standby_;
  bool is_goal_accepted_;
  bool is_waypoints_updated_;
  bool is_canceled_;
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