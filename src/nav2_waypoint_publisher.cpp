#include "nav2_waypoint_publisher/nav2_waypoint_publisher.hpp"  // include local header

WayPointPublisher::WayPointPublisher() : rclcpp::Node("nav2_waypoint_publisher"), id_(0)
{
  declareParams();
  getParams();
  rclcpp::QoS latched_qos{ 1 };
  latched_qos.transient_local();
  waypoint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", latched_qos);
  waypoint_text_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_index", latched_qos);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&WayPointPublisher::JoyCallback, this, std::placeholders::_1));
  std::string action_server_name;

  if (follow_type_ == THROUGH_POSES_MODE){
    action_server_name = "navigate_through_poses";
    nav_through_poses_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, action_server_name);
    rclcpp::sleep_for(500ms);
    is_action_server_ready_ = nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));

  }else if (follow_type_ == FOLLOW_WAYPOITNS_MODE){
    action_server_name = "follow_waypoints";
    follow_waypoints_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this, action_server_name);
    rclcpp::sleep_for(500ms);
    is_action_server_ready_ = follow_waypoints_action_client_->wait_for_action_server(std::chrono::seconds(5));
  }
  send_goal_options_ = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options_.result_callback = std::bind(&WayPointPublisher::NavThroughPosesResultCallback, this, std::placeholders::_1);
  send_goal_options_.goal_response_callback = std::bind(&WayPointPublisher::NavThroughPosesGoalResponseCallback, this, std::placeholders::_1);

  ReadWaypointsFromCSV(csv_file_, waypoints_);
  start_index_ = (size_t)start_index_int_;
  if(start_index_ < 1 || start_index_ > waypoints_.size()){
    RCLCPP_ERROR(get_logger(), "Invalid start_index");
    return;
  }

  PublishWaypointMarkers(waypoints_, start_index_);
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
}

void WayPointPublisher::declareParams()
{
  follow_type_ = declare_parameter<int>("follow_type", FOLLOW_WAYPOITNS_MODE);
  start_index_int_ = declare_parameter<int>("start_index", 1);
  csv_file_ = declare_parameter<std::string>("csv_file", "sample.csv");
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
  bool GotFllowType = this->get_parameter("follow_type", follow_type_);
  bool GotCSVFile = this->get_parameter("csv_file", csv_file_);
  bool StartIndex = this->get_parameter("start_index", start_index_);
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
  bool pass = checkParameters({ GotFllowType, GotCSVFile, GotWayPointMarkerScale, GotWayPointMarkerColorR,StartIndex,
                                GotWayPointMarkerColorG, GotWayPointMarkerColorB, GotWayPointMarkerColorA,GotWayPointStopMarkerColorR,
                                GotWayPointStopMarkerColorG,GotWayPointStopMarkerColorB,GotWayPointStopMarkerColorA,
                                GotWayPointTextMarkerScale, GotWayPointTextMarkerColorR, GotWayPointTextMarkerColorG,
                                GotWayPointTextMarkerColorB, GotWayPointTextMarkerColorA });
  if (!pass)
  {
    RCLCPP_WARN(get_logger(), "Could not get type paramters. Use default parameters");
  }
  if (follow_type_ != FOLLOW_WAYPOITNS_MODE && follow_type_ != THROUGH_POSES_MODE)
  {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "follow_type param has to be 0 or 1. Current the param value is " << follow_type_);
  }
}

geometry_msgs::msg::Quaternion rpyYawToQuat(double yaw){
  tf2::Quaternion tf_quat;
  geometry_msgs::msg::Quaternion msg_quat;
  tf_quat.setRPY(0.0, 0.0 ,yaw);
  msg_quat = tf2::toMsg(tf_quat);
  return msg_quat;
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

void WayPointPublisher::ReadWaypointsFromCSV(std::string& csv_file, std::vector<waypoint_info>& waypoints){
  std::ifstream ifs(csv_file);
  std::string line;
  waypoint_info waypoint;
  int id = 0;
  while (getline(ifs, line))
  {
    id++;
    std::vector<std::string> strvec = getCSVLine(line, ',');
    waypoint.poses.position.x = std::stod(strvec.at(0));
    waypoint.poses.position.y = std::stod(strvec.at(1));
    waypoint.poses.position.z = 0.0;
    waypoint.poses.orientation = rpyYawToQuat(std::stod(strvec.at(2))/180.0*M_PI);
    waypoint.will_stop = ("1"==strvec.at(5));

    // std::cout << "-------------------------------------" << std::endl;
    // std::cout << "waypoint ID: " << id << std::endl;
    // std::cout << "trans x: " << std::stod(strvec.at(0)) << std::endl;
    // std::cout << "trans y: " << std::stod(strvec.at(1)) << std::endl;
    // std::cout << "rot yaw: " << std::stod(strvec.at(2)) << std::endl;
    // std::cout << "will stop: "<< std::boolalpha <<  ("1"==strvec.at(5)) << std::endl;

    waypoints.push_back(waypoint);
  }
}

void WayPointPublisher::PublishWaypointMarkers(const std::vector<waypoint_info> waypoints, size_t start_index){
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

  for(i=start_index; i<waypoints.size(); i++){
    visualization_msgs::msg::Marker marker, text_marker;
    marker = origin_marker;
    text_marker = origin_text_marker;

    if(waypoints[i-1].will_stop){  //止まるウェイポイントだったら色を変える
      marker.color.r = waypoint_stop_marker_color_r_;
      marker.color.g = waypoint_stop_marker_color_g_;
      marker.color.b = waypoint_stop_marker_color_b_;
      marker.color.a = waypoint_stop_marker_color_a_;
    }

    marker.id = i;
    marker.pose = waypoints[i-1].poses;

    text_marker.id = i;
    text_marker.text = std::to_string(i);
    text_marker.pose = waypoints[i-1].poses;

    marker_array.markers.push_back(marker);
    text_marker_array.markers.push_back(text_marker);
  }
  waypoint_pub_->publish(marker_array);
  waypoint_text_pub_->publish(text_marker_array);
}

void WayPointPublisher::SendWaypointsTimerCallback(){
  static size_t sending_index = start_index_ - 1;
  static int state = SEND_WAYPOINTS;

  switch (state)
  {
  case SEND_WAYPOINTS:
    if(sending_index < waypoints_.size()){
      sending_index =  SendWaypointsOnce(sending_index);
      state = SEND_WAYPOINTS_CHECK;
    }else{
      state = FINISH_SENDING;
    }
    break;

  case SEND_WAYPOINTS_CHECK:
      if(follow_type_ == THROUGH_POSES_MODE){
        if(is_goal_accepted_){
          state = WAITING_GOAL;
          RCLCPP_INFO(this->get_logger(), "Waiting to acheive goal.");
        }
      }

      if(follow_type_ == FOLLOW_WAYPOITNS_MODE){
        state = FINISH_SENDING;
      }
    break;

  case WAITING_GOAL:
    if(is_goal_achieved_){
      is_standby_ = false;
      state = WAITING_BUTTON;
      RCLCPP_INFO(this->get_logger(), "Waiting start botton.");
    }
    break;
  
  case WAITING_BUTTON:
    if(is_standby_){
      state = SEND_WAYPOINTS;
    }
    break;
  
  case FINISH_SENDING:
    RCLCPP_INFO(this->get_logger(), "Waypoint sending is Finisihed.");
    timer_->cancel();
  break;

  default:
    RCLCPP_INFO(this->get_logger(), "UNKNOWN ERROR");
    timer_->cancel();
    break;
  }
}
size_t WayPointPublisher::SendWaypointsOnce(size_t sending_index){
  size_t i;
  size_t next_index;
  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal;
  nav2_msgs::action::FollowWaypoints::Goal follow_waypoints_goal;
  for(i = sending_index; i<waypoints_.size(); i++){
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose=waypoints_[i].poses;
    nav_through_poses_goal.poses.push_back(goal_msg);
    follow_waypoints_goal.poses.push_back(goal_msg);
    if(follow_type_ == THROUGH_POSES_MODE && waypoints_[i].will_stop)break;
  }
  next_index = i+1;
  if (follow_type_ == THROUGH_POSES_MODE){
    is_goal_achieved_ = false;
    is_goal_accepted_ = false;

    std::chrono::milliseconds server_timeout(1000);
    future_goal_handle_ = nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal, send_goal_options_);
    RCLCPP_INFO(this->get_logger(),
              "[nav_through_poses]: Sending a path of %zu waypoints:", nav_through_poses_goal.poses.size());
  }

  if (follow_type_ == FOLLOW_WAYPOITNS_MODE){
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) { follow_waypoints_goal_handle_.reset(); };

    std::chrono::milliseconds server_timeout(1000);
    auto future_goal_handle =
        follow_waypoints_action_client_->async_send_goal(follow_waypoints_goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle, server_timeout) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
      //return;
    }
    // Get the goal handle and save so that we can check on completion in the timer callback
    follow_waypoints_goal_handle_ = future_goal_handle.get();
    if (!follow_waypoints_goal_handle_)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    RCLCPP_INFO(this->get_logger(),
                "[follow_waypoints]: Sending a path of %zu waypoints:", follow_waypoints_goal.poses.size());
  }
  return next_index;
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
      return;
    
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
}
void WayPointPublisher::NavThroughPosesGoalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>> future){
  auto handle = future.get();
  if (!handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    timer_->cancel();
    return ;
  }
  is_goal_accepted_ = true; 
}
void WayPointPublisher::JoyCallback(const sensor_msgs::msg::Joy &joy_msg){
  static bool was_pushed = false; 
  if (joy_msg.buttons[0] == 1){
      if(!was_pushed){
          RCLCPP_INFO(this->get_logger(), "start_botton is pressed.");
          was_pushed = true;
          is_standby_ = true;
      }
  }else{
      was_pushed = false;
  }
}