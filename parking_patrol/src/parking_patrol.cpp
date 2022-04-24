#include <parking_patrol/ros/spot_finder_node.hpp>
#include <parking_patrol/ros/parking_patrol_node.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace parkingpatrol;

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  std::cout << "|=======================================|\n"
               "|   Parking Patrol   ||   Pawel Irzyk   |\n"
               "|=======================================|\n"
               "[ParkingPatrol] Initializing ROS..." << std::endl;

  rclcpp::executors::MultiThreadedExecutor executor;
  auto spot_finder_node = std::make_shared<SpotFinderNode>();
  // auto parking_patrol_node = std::make_shared<ParkingPatrolNode>(spot_finder_node);
  executor.add_node(spot_finder_node);
  // executor.add_node(parking_patrol_node);
  try {
    executor.spin();
  } catch (std::runtime_error &err){
    std::cout << "[ParkingPatrolNode] Caught exception: " << err.what() << std::endl;
  }

  std::cout << "[ParkingPatrolNode] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}
