#include "nav2_waypoint_publisher/nav2_waypoint_publisher.hpp"  // include local header

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WayPointPublisher>());
  rclcpp::shutdown();

  return 0;
}