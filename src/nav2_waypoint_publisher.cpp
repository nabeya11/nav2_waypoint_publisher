#include "nav2_waypoint_publisher/nav2_waypoint_publisher.hpp"  // include local header

WayPointPublisher::WayPointPublisher() : rclcpp::Node("nav2_waypoint_publisher"), id_(0)
{
  declareParams();
  getParams();
  rclcpp::QoS latched_qos{ 1 };
  latched_qos.transient_local();
  waypoint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", latched_qos);
  waypoint_text_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_index", latched_qos);
  if (follow_type_ == 0)
  {
    nav_through_poses_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");
    is_action_server_ready_ = nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready_)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "navigate_through_poses action server is not available."
                   " Is the initial pose set?");
      return;
    }
  }
  else if (follow_type_ == 1)
  {
    follow_waypoints_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this, "follow_waypoints");
    is_action_server_ready_ = follow_waypoints_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready_)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "follow_waypoints action server is not available."
                   " Is the initial pose set?");
      return;
    }
  }
  publishWaypointsFromCSV(csv_file_);
}

void WayPointPublisher::declareParams()
{
  follow_type_ = declare_parameter<int>("follow_type", 1);
  csv_file_ = declare_parameter<std::string>("csv_file", "sample.csv");
  waypoint_marker_scale_ = declare_parameter<float>("waypoint_marker_scale", 0.5);
  waypoint_marker_color_r_ = declare_parameter<float>("waypoint_marker_color_r", 1.0f);
  waypoint_marker_color_g_ = declare_parameter<float>("waypoint_marker_color_g", 0.0f);
  waypoint_marker_color_b_ = declare_parameter<float>("waypoint_marker_color_b", 0.0f);
  waypoint_marker_color_a_ = declare_parameter<float>("waypoint_marker_color_a", 1.0f);
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
  bool GotWayPointMarkerScale = this->get_parameter("waypoint_marker_scale", waypoint_marker_scale_);
  bool GotWayPointMarkerColorR = this->get_parameter("waypoint_marker_color_r", waypoint_marker_color_r_);
  bool GotWayPointMarkerColorG = this->get_parameter("waypoint_marker_color_g", waypoint_marker_color_g_);
  bool GotWayPointMarkerColorB = this->get_parameter("waypoint_marker_color_b", waypoint_marker_color_b_);
  bool GotWayPointMarkerColorA = this->get_parameter("waypoint_marker_color_a", waypoint_marker_color_a_);
  bool GotWayPointTextMarkerScale = this->get_parameter("waypoint_text_marker_scale", waypoint_text_marker_scale_);
  bool GotWayPointTextMarkerColorR = this->get_parameter("waypoint_text_marker_color_r", waypoint_text_marker_color_r_);
  bool GotWayPointTextMarkerColorG = this->get_parameter("waypoint_text_marker_color_g", waypoint_text_marker_color_g_);
  bool GotWayPointTextMarkerColorB = this->get_parameter("waypoint_text_marker_color_b", waypoint_text_marker_color_b_);
  bool GotWayPointTextMarkerColorA = this->get_parameter("waypoint_text_marker_color_a", waypoint_text_marker_color_a_);
  bool pass = checkParameters({ GotFllowType, GotCSVFile, GotWayPointMarkerScale, GotWayPointMarkerColorR,
                                GotWayPointMarkerColorG, GotWayPointMarkerColorB, GotWayPointMarkerColorA,
                                GotWayPointTextMarkerScale, GotWayPointTextMarkerColorR, GotWayPointTextMarkerColorG,
                                GotWayPointTextMarkerColorB, GotWayPointTextMarkerColorA });
  if (pass)
  {
    RCLCPP_WARN(get_logger(), "Could not get type paramters. Use default parameters");
  }
  if (follow_type_ > 1 || follow_type_ < 0)
  {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "follow_type param has to be 0 or 1. Current the param value is " << follow_type_);
  }
}

bool WayPointPublisher::checkParameters(const std::vector<bool>& list)
{
  bool pass = true;

  for (int i = 0; i < list.size(); ++i)
  {
    if (!list[i])
    {
      pass = false;
      std::cout << "didn't get i: " << i << " in the launch file" << std::endl;
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

void WayPointPublisher::publishWaypointsFromCSV(std::string csv_file)
{
  std::ifstream ifs(csv_file);
  std::string line;
  visualization_msgs::msg::MarkerArray marker_array, text_marker_array;
  visualization_msgs::msg::Marker origin_marker;
  visualization_msgs::msg::Marker origin_text_marker;
  origin_marker.header.frame_id = "map";
  origin_marker.header.stamp = this->now();
  origin_marker.ns = "waypoints_marker";
  origin_marker.id = 0;
  origin_marker.type = visualization_msgs::msg::Marker::ARROW;
  origin_marker.action = visualization_msgs::msg::Marker::ADD;
  origin_marker.lifetime = rclcpp::Duration(0);  // forever
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
  origin_text_marker.lifetime = rclcpp::Duration(0);  // forever
  origin_text_marker.scale.x = waypoint_text_marker_scale_;
  origin_text_marker.scale.y = waypoint_text_marker_scale_;
  origin_text_marker.scale.z = waypoint_text_marker_scale_;
  origin_text_marker.color.r = waypoint_text_marker_color_r_;
  origin_text_marker.color.g = waypoint_text_marker_color_g_;
  origin_text_marker.color.b = waypoint_text_marker_color_b_;
  origin_text_marker.color.a = waypoint_text_marker_color_a_;

  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal;
  nav2_msgs::action::FollowWaypoints::Goal follow_waypoints_goal;

  while (getline(ifs, line))
  {
    std::vector<std::string> strvec = getCSVLine(line, ',');
    geometry_msgs::msg::PoseStamped goal_msg;
    visualization_msgs::msg::Marker marker, text_marker;
    marker = origin_marker;
    text_marker = origin_text_marker;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";

    goal_msg.pose.position.x = std::stod(strvec.at(0));
    goal_msg.pose.position.y = std::stod(strvec.at(1));
    goal_msg.pose.orientation.x = std::stod(strvec.at(3));
    goal_msg.pose.orientation.y = std::stod(strvec.at(4));
    goal_msg.pose.orientation.z = std::stod(strvec.at(5));
    goal_msg.pose.orientation.w = std::stod(strvec.at(6));

    marker.id = id_++;
    marker.pose.position.x = std::stod(strvec.at(0));
    marker.pose.position.y = std::stod(strvec.at(1));
    marker.pose.position.z = std::stod(strvec.at(2));
    marker.pose.orientation.x = std::stod(strvec.at(3));
    marker.pose.orientation.y = std::stod(strvec.at(4));
    marker.pose.orientation.z = std::stod(strvec.at(5));
    marker.pose.orientation.w = std::stod(strvec.at(6));

    text_marker.id = id_;
    text_marker.text = std::to_string(id_);
    text_marker.pose.position.x = std::stod(strvec.at(0));
    text_marker.pose.position.y = std::stod(strvec.at(1));
    text_marker.pose.position.z = std::stod(strvec.at(2));
    text_marker.pose.orientation.x = std::stod(strvec.at(3));
    text_marker.pose.orientation.y = std::stod(strvec.at(4));
    text_marker.pose.orientation.z = std::stod(strvec.at(5));
    text_marker.pose.orientation.w = std::stod(strvec.at(6));

    std::cout << "-------------------------------------" << std::endl;
    std::cout << "waypoint ID: " << marker.id << std::endl;
    std::cout << "trans x: " << std::stod(strvec.at(0)) << std::endl;
    std::cout << "trans y: " << std::stod(strvec.at(1)) << std::endl;
    std::cout << "trans z: " << std::stod(strvec.at(2)) << std::endl;
    std::cout << "rot x: " << std::stod(strvec.at(3)) << std::endl;
    std::cout << "rot y: " << std::stod(strvec.at(4)) << std::endl;
    std::cout << "rot z: " << std::stod(strvec.at(5)) << std::endl;
    std::cout << "rot w: " << std::stod(strvec.at(6)) << std::endl;
    if (follow_type_ == 0)
      nav_through_poses_goal.poses.push_back(goal_msg);
    else if (follow_type_ == 1)
      follow_waypoints_goal.poses.push_back(goal_msg);

    marker_array.markers.push_back(marker);
    text_marker_array.markers.push_back(text_marker);
  }

  // send goal
  if (follow_type_ == 0)
  {
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) { nav_through_poses_goal_handle_.reset(); };

    std::chrono::milliseconds server_timeout(30);
    auto future_goal_handle =
        nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle, server_timeout) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
      return;
    }
    // Get the goal handle and save so that we can check on completion in the timer callback
    nav_through_poses_goal_handle_ = future_goal_handle.get();
    if (!nav_through_poses_goal_handle_)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "[nav_through_poses]: Sending a path of %zu waypoints:", nav_through_poses_goal.poses.size());
  }

  if (follow_type_ == 1)
  {
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) { follow_waypoints_goal_handle_.reset(); };

    std::chrono::milliseconds server_timeout(30);
    auto future_goal_handle =
        follow_waypoints_action_client_->async_send_goal(follow_waypoints_goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle, server_timeout) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
      return;
    }
    // Get the goal handle and save so that we can check on completion in the timer callback
    follow_waypoints_goal_handle_ = future_goal_handle.get();
    if (!follow_waypoints_goal_handle_)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "[follow_waypoints]: Sending a path of %zu waypoints:", follow_waypoints_goal.poses.size());
  }

  // Publish waypoints markers
  waypoint_pub_->publish(marker_array);
  waypoint_text_pub_->publish(text_marker_array);
}
