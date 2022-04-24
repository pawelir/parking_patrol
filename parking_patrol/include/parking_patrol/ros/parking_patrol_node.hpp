#ifndef PARKING_PATROL_PARKING_PATROL_NODE_HPP
#define PARKING_PATROL_PARKING_PATROL_NODE_HPP

#include <parking_patrol/ros/spot_finder_node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <parking_patrol_msgs/srv/free_spots.hpp>

namespace parkingpatrol {

using SrvTrigger = std_srvs::srv::Trigger;
using SrvFreeSpots = parking_patrol_msgs::srv::FreeSpots;

enum StatusCode{
  REJECTED,
  SUCCEDED
};

class ParkingPatrolNode : rclcpp::Node {

 public:
  ParkingPatrolNode(std::shared_ptr<SpotFinderNode> spot_finder_node);

 private:
  void restart_patrol_cb_(const std::shared_ptr<SrvTrigger::Request>,
                          const std::shared_ptr<SrvTrigger::Response> res);

  void finish_patrol_cb_(const std::shared_ptr<SrvFreeSpots::Request>,
                         const std::shared_ptr<SrvFreeSpots::Response> res);

  std::vector<MsgPose> filter_multiple_poses(const std::vector<MsgPose> &poses);
  
  std::shared_ptr<SpotFinderNode> spot_finder_node_;
  rclcpp::Service<SrvTrigger>::SharedPtr srv_restart_patrol_;
  rclcpp::Service<SrvFreeSpots>::SharedPtr srv_finish_patrol_;

  rclcpp::CallbackGroup::SharedPtr cb_group_reeterant_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mutually_exclusive_;
};

}// namespace parkingpatrol

#endif//PARKING_PATROL_PARKING_PATROL_NODE_HPP
