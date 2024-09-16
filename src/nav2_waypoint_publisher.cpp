#include "nav2_waypoint_publisher/nav2_waypoint_publisher.hpp"  // include local header

WayPointPublisher::WayPointPublisher() : rclcpp::Node("nav2_waypoint_publisher"), id_(0)
{
  declareParams();
  getParams();
  rclcpp::QoS latched_qos{ 1 };
  latched_qos.transient_local();
  waypoint_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_marker", latched_qos);
  waypoint_text_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_maker_index", latched_qos);
  waypoint_state_pub_ = create_publisher<tsukutsuku2_msgs::msg::StateAndFeedback>("waypoint_state", latched_qos);
  waypoints_sub_ = create_subscription<tsukutsuku2_msgs::msg::Waypoints>("/waypoints", 1, std::bind(&WayPointPublisher::WaypointsSubCallback, this, std::placeholders::_1));
  std::string action_server_name;

  action_server_name = "navigate_through_poses";
  nav_through_poses_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, action_server_name);
  rclcpp::sleep_for(500ms);
  is_action_server_ready_ = nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));


  send_goal_options_ = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options_.feedback_callback = std::bind(&WayPointPublisher::NavThroughPosesFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options_.result_callback = std::bind(&WayPointPublisher::NavThroughPosesResultCallback, this, std::placeholders::_1);
  send_goal_options_.goal_response_callback = std::bind(&WayPointPublisher::NavThroughPosesGoalResponseCallback, this, std::placeholders::_1);

  if (is_action_server_ready_){
     timer_ = create_wall_timer(100ms, std::bind(&WayPointPublisher::SendWaypointsTimerCallback, this));

  }else{
    RCLCPP_ERROR(this->get_logger(),
                  "%s action server is not available."
                  " Is the initial pose set?"
                  " SendWaypoints was not executed.",
                  action_server_name.c_str());
    return;
  }
  is_waypoints_updated_ = false;
}

void WayPointPublisher::declareParams()
{
  waypoint_marker_scale_ = declare_parameter<float>("waypoint_marker_scale", 0.5);
  waypoint_marker_color_r_ = declare_parameter<float>("waypoint_marker_color_r", 1.0f);
  waypoint_marker_color_g_ = declare_parameter<float>("waypoint_marker_color_g", 0.0f);
  waypoint_marker_color_b_ = declare_parameter<float>("waypoint_marker_color_b", 0.0f);
  waypoint_marker_color_a_ = declare_parameter<float>("waypoint_marker_color_a", 1.0f);
  waypoint_stop_marker_color_r_ = declare_parameter<float>("waypoint_stop_marker_color_r", 1.0f);
  waypoint_stop_marker_color_g_ = declare_parameter<float>("waypoint_stop_marker_color_g", 0.0f);
  waypoint_stop_marker_color_b_ = declare_parameter<float>("waypoint_stop_marker_color_b", 0.0f);
  waypoint_stop_marker_color_a_ = declare_parameter<float>("waypoint_stop_marker_color_a", 1.0f);
  waypoint_text_marker_scale_ = declare_parameter<float>("waypoint_text_marker_scale", 0.5);
  waypoint_text_marker_color_r_ = declare_parameter<float>("waypoint_text_marker_color_r", 1.0f);
  waypoint_text_marker_color_g_ = declare_parameter<float>("waypoint_text_marker_color_g", 0.0f);
  waypoint_text_marker_color_b_ = declare_parameter<float>("waypoint_text_marker_color_b", 0.0f);
  waypoint_text_marker_color_a_ = declare_parameter<float>("waypoint_text_marker_color_a", 1.0f);
}

void WayPointPublisher::getParams()
{
  bool GotWayPointMarkerScale = this->get_parameter("waypoint_marker_scale", waypoint_marker_scale_);
  bool GotWayPointMarkerColorR = this->get_parameter("waypoint_marker_color_r", waypoint_marker_color_r_);
  bool GotWayPointMarkerColorG = this->get_parameter("waypoint_marker_color_g", waypoint_marker_color_g_);
  bool GotWayPointMarkerColorB = this->get_parameter("waypoint_marker_color_b", waypoint_marker_color_b_);
  bool GotWayPointMarkerColorA = this->get_parameter("waypoint_marker_color_a", waypoint_marker_color_a_);
  bool GotWayPointStopMarkerColorR = this->get_parameter("waypoint_stop_marker_color_r", waypoint_stop_marker_color_r_);
  bool GotWayPointStopMarkerColorG = this->get_parameter("waypoint_stop_marker_color_g", waypoint_stop_marker_color_g_);
  bool GotWayPointStopMarkerColorB = this->get_parameter("waypoint_stop_marker_color_b", waypoint_stop_marker_color_b_);
  bool GotWayPointStopMarkerColorA = this->get_parameter("waypoint_stop_marker_color_a", waypoint_stop_marker_color_a_);
  bool GotWayPointTextMarkerScale = this->get_parameter("waypoint_text_marker_scale", waypoint_text_marker_scale_);
  bool GotWayPointTextMarkerColorR = this->get_parameter("waypoint_text_marker_color_r", waypoint_text_marker_color_r_);
  bool GotWayPointTextMarkerColorG = this->get_parameter("waypoint_text_marker_color_g", waypoint_text_marker_color_g_);
  bool GotWayPointTextMarkerColorB = this->get_parameter("waypoint_text_marker_color_b", waypoint_text_marker_color_b_);
  bool GotWayPointTextMarkerColorA = this->get_parameter("waypoint_text_marker_color_a", waypoint_text_marker_color_a_);
  bool pass = checkParameters({ GotWayPointMarkerScale, GotWayPointMarkerColorR,
                                GotWayPointMarkerColorG, GotWayPointMarkerColorB, GotWayPointMarkerColorA,GotWayPointStopMarkerColorR,
                                GotWayPointStopMarkerColorG,GotWayPointStopMarkerColorB,GotWayPointStopMarkerColorA,
                                GotWayPointTextMarkerScale, GotWayPointTextMarkerColorR, GotWayPointTextMarkerColorG,
                                GotWayPointTextMarkerColorB, GotWayPointTextMarkerColorA });
  if (!pass)
  {
    RCLCPP_WARN(get_logger(), "Could not get type paramters. Use default parameters");
  }
}

bool WayPointPublisher::checkParameters(const std::vector<bool>& list)
{
  bool pass = true;

  for (size_t i = 0; i < list.size(); ++i)
  {
    if (!list[i])
    {
      pass = false;
      RCLCPP_WARN_STREAM(get_logger(),"didn't get i: " << i << " in the launch file");
    }
  }

  return pass;
}

std::vector<std::string> WayPointPublisher::getCSVLine(std::string& input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (getline(stream, field, delimiter))
  {
    result.push_back(field);
  }
  return result;
}

void WayPointPublisher::PublishWaypointMarkers(){
  size_t i;
  visualization_msgs::msg::MarkerArray marker_array, text_marker_array;
  visualization_msgs::msg::Marker origin_marker;
  visualization_msgs::msg::Marker origin_text_marker;
  origin_marker.header.frame_id = "map";
  origin_marker.header.stamp = this->now();
  origin_marker.ns = "waypoints_marker";
  origin_marker.id = 0;
  origin_marker.type = visualization_msgs::msg::Marker::ARROW;
  origin_marker.action = visualization_msgs::msg::Marker::ADD;
  origin_marker.lifetime = rclcpp::Duration(0, 0);  // forever
  origin_marker.scale.x = waypoint_marker_scale_ * 1.5;
  origin_marker.scale.y = waypoint_marker_scale_;
  origin_marker.scale.z = waypoint_marker_scale_;
  origin_marker.color.r = waypoint_marker_color_r_;
  origin_marker.color.g = waypoint_marker_color_g_;
  origin_marker.color.b = waypoint_marker_color_b_;
  origin_marker.color.a = waypoint_marker_color_a_;

  origin_text_marker.header.frame_id = "map";
  origin_text_marker.header.stamp = this->now();
  origin_text_marker.ns = "waypoints_text_marker";
  origin_text_marker.id = 0;
  origin_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  origin_text_marker.action = visualization_msgs::msg::Marker::ADD;
  origin_text_marker.lifetime = rclcpp::Duration(0, 0);  // forever
  origin_text_marker.scale.x = waypoint_text_marker_scale_;
  origin_text_marker.scale.y = waypoint_text_marker_scale_;
  origin_text_marker.scale.z = waypoint_text_marker_scale_;
  origin_text_marker.color.r = waypoint_text_marker_color_r_;
  origin_text_marker.color.g = waypoint_text_marker_color_g_;
  origin_text_marker.color.b = waypoint_text_marker_color_b_;
  origin_text_marker.color.a = waypoint_text_marker_color_a_;

  for(i=1; i<waypoints_.size()+1; i++){
    visualization_msgs::msg::Marker marker, text_marker;
    marker = origin_marker;
    text_marker = origin_text_marker;

    if(waypoints_[i-1].state != NORMAL_RUNNING_MODE){  //止まるウェイポイントだったら色を変える
      marker.color.r = waypoint_stop_marker_color_r_;
      marker.color.g = waypoint_stop_marker_color_g_;
      marker.color.b = waypoint_stop_marker_color_b_;
      marker.color.a = waypoint_stop_marker_color_a_;
    }

    marker.id = i;
    marker.pose = waypoints_[i-1].pose;

    text_marker.id = i;
    text_marker.text = std::to_string(i);
    text_marker.pose = waypoints_[i-1].pose;

    marker_array.markers.push_back(marker);
    text_marker_array.markers.push_back(text_marker);
  }
  waypoint_marker_pub_->publish(marker_array);
  waypoint_text_pub_->publish(text_marker_array);
}

void WayPointPublisher::SendWaypointsTimerCallback(){
  static int state = STANBY;
  static uint8_t feedback_state = FEEDBACK_STANBY;
  tsukutsuku2_msgs::msg::StateAndFeedback feedback_state_msg;
  feedback_state_msg.state = latest_waypoint_state_;

  switch (state)
  {
  case SEND_WAYPOINTS:
    SendWaypointsOnce();
    PublishWaypointMarkers();
    state = SEND_WAYPOINTS_CHECK;
    feedback_state = FEEDBACK_PROCCESSING; 
    is_checked_ = false;
    break;

  case SEND_WAYPOINTS_CHECK:
      if(is_checked_){
        if(is_goal_accepted_){
          state = WAITING_GOAL;
          RCLCPP_INFO(this->get_logger(), "Waiting to acheive goal.");
        }else{
          feedback_state = FEEDBACK_REJECTED;
          state = STANBY;
        }
      }
      
    break;

  case WAITING_GOAL:
    if(is_waypoints_updated_){
      state = CANCEL_GOAL;
      feedback_state = FEEDBACK_CANCELING;
      break;
    }
    if(is_goal_achieved_){
      is_standby_ = false;
      state = STANBY;
      feedback_state = FEEDBACK_STANBY;
      RCLCPP_INFO(this->get_logger(), 
                              "Goal is achived."
                              "Waypoint Publisher is now in standby mode."
                              "Waiting next waypoints."
                              );
    }
    if(is_aborted_){
      waypoints_.erase(waypoints_.begin());
      RCLCPP_WARN(this->get_logger(), "Skipping current waypoint. Restarting from the next waypoint...");
      state = SEND_WAYPOINTS;
      feedback_state = FEEDBACK_ABORTED;
    }
    break;
  
  case STANBY:
    if(is_waypoints_updated_){
      state = SEND_WAYPOINTS;
      break;
    }
  break;

  case CANCEL_GOAL:
    RCLCPP_INFO(this->get_logger(), "Waypoints have been updated.");
    is_canceled_ = false;
    CancelTask();
    state = WAITING_CANCEL;
  break;

  case WAITING_CANCEL:
    if(is_canceled_){
      RCLCPP_INFO(this->get_logger(), "Task have been canceled.");
      state = SEND_WAYPOINTS;
    }
  break;

  default:
    RCLCPP_ERROR(this->get_logger(), "UNKNOWN ERROR");
    timer_->cancel();
    break;
  }
  feedback_state_msg.feed_back = feedback_state;
  waypoint_state_pub_->publish(feedback_state_msg);
}
void WayPointPublisher::SendWaypointsOnce(){
  size_t i;
  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal;
  for(i = 0; i<waypoints_.size(); i++){
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose=waypoints_[i].pose;
    nav_through_poses_goal.poses.push_back(goal_msg);
  }
    is_goal_achieved_ = false;
    is_goal_accepted_ = false;
    is_aborted_ = false;
    is_waypoints_updated_ = false;

    std::chrono::milliseconds server_timeout(1000);
    future_goal_handle_ = nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal, send_goal_options_);
    RCLCPP_INFO(this->get_logger(),
              "[nav_through_poses]: Sending a path of %zu waypoints:", nav_through_poses_goal.poses.size());
  return;
}
void WayPointPublisher::CancelTask(){
  nav_through_poses_action_client_->async_cancel_all_goals();
  
}
void WayPointPublisher::NavThroughPosesResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result){
  nav_through_poses_goal_handle_.reset();
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      is_goal_achieved_ = true;
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      is_aborted_ = true;
      return;
    
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      is_canceled_ = true;
      return;
    
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
}
void WayPointPublisher::NavThroughPosesGoalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>> future){
  is_checked_ = true;
  auto handle = future.get();
  if (!handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    is_goal_accepted_ = false; 
    return ;
  }
  is_goal_accepted_ = true; 
}
void WayPointPublisher::NavThroughPosesFeedbackCallback(const GoalHandleNavigateNavigateThroughPoses::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback){
  number_of_poses_remaining_ = feedback->number_of_poses_remaining;
  //RCLCPP_INFO(get_logger(), "number of poses remaining = %zu", (size_t)feedback->number_of_poses_remaining);
}
void WayPointPublisher::WaypointsSubCallback(const tsukutsuku2_msgs::msg::Waypoints::SharedPtr waypoints_msg){
  waypoints_ = waypoints_msg->waypoints;
  latest_waypoint_state_ = waypoints_.back().state;
  is_waypoints_updated_ = true;
}