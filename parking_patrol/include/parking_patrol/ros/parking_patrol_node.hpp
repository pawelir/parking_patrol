#ifndef PARKING_PATROL_PARKING_PATROL_NODE_HPP
#define PARKING_PATROL_PARKING_PATROL_NODE_HPP

#include <cmath>
#include <optional>

#include <parking_patrol_msgs/srv/free_spots.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <parking_patrol/ros/spot_finder_node.hpp>

namespace parkingpatrol {
using MsgPoint = geometry_msgs::msg::Point;
using SrvFreeSpots = parking_patrol_msgs::srv::FreeSpots;
using SrvTrigger = std_srvs::srv::Trigger;

class ParkingPatrolNode : public rclcpp::Node {
public:
  ParkingPatrolNode(std::shared_ptr<SpotFinderNode> spot_finder_node);

private:
  void restart_patrol_cb_(const std::shared_ptr<SrvTrigger::Request>,
                          const std::shared_ptr<SrvTrigger::Response> res);

  void finish_patrol_cb_(const std::shared_ptr<SrvFreeSpots::Request>,
                         const std::shared_ptr<SrvFreeSpots::Response> res);

  std::map<std::string, float> orientation_map_{
      {direction::UP, 0.0},
      {direction::DOWN, 1.0},
      {direction::TOLERANCE, 0.06},
  };

  std::pair<std::vector<MsgPoint>, std::vector<MsgPoint>>
  match_detections_to_columns(
      const std::unordered_map<std::string, std::vector<SpotDetectionPose>>
          &detections);

  MsgPoint msg_point_from_detection(const SpotDetectionPose &detection);

  std::shared_ptr<SpotFinderNode> spot_finder_node_;
  rclcpp::Service<SrvTrigger>::SharedPtr srv_restart_patrol_;
  rclcpp::Service<SrvFreeSpots>::SharedPtr srv_finish_patrol_;

  rclcpp::CallbackGroup::SharedPtr cb_group_mutually_exclusive_;
};

} // namespace parkingpatrol

#endif // PARKING_PATROL_PARKING_PATROL_NODE_HPP
