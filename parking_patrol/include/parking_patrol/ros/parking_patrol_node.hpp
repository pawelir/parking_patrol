#ifndef PARKING_PATROL_PARKING_PATROL_NODE_HPP
#define PARKING_PATROL_PARKING_PATROL_NODE_HPP

#include <parking_patrol/ros/spot_finder_node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>
#include <optional>
#include <parking_patrol_msgs/srv/free_spots.hpp>

namespace parkingpatrol {

using SrvTrigger = std_srvs::srv::Trigger;
using SrvFreeSpots = parking_patrol_msgs::srv::FreeSpots;
using MsgPoint = geometry_msgs::msg::Point;

enum StatusCode{
  REJECTED,
  SUCCEDED
};

static constexpr char UPSIDE[] = "UPSIDE";
static constexpr char DOWNSIDE[] = "DOWNSIDE";
static constexpr char TOLERANCE[] = "TOLERANCE";
namespace column{
static constexpr char LEFT[] = "LEFT";
static constexpr char RIGHT[] = "RIGHT";
}//namespace column

class ParkingPatrolNode : public rclcpp::Node {

 public:
  ParkingPatrolNode(std::shared_ptr<SpotFinderNode> spot_finder_node);

 private:
  void declare_parameters();
  
  void load_parameters();

  void restart_patrol_cb_(const std::shared_ptr<SrvTrigger::Request>,
                          const std::shared_ptr<SrvTrigger::Response> res);

  void finish_patrol_cb_(const std::shared_ptr<SrvFreeSpots::Request>,
                         const std::shared_ptr<SrvFreeSpots::Response> res);

  float spot_width_;
  float parking_spot_increment_;

  std::map<std::string, float> orientation_map_ {
    {UPSIDE, 0.0},
    {DOWNSIDE, -1.0},
    {TOLERANCE, 0.1},
  };

  std::vector<SpotDetectionPose> filter_multiple_detections(std::vector<SpotDetectionPose> &detections);

  std::pair<std::vector<MsgPoint>, std::vector<MsgPoint>> match_detections_to_columns(
      const std::map<std::string, std::vector<SpotDetectionPose>> &detections);

  MsgPoint msg_point_from_detection(const SpotDetectionPose &detection);


  std::vector<MsgPoint> match_orientation(const std::vector<SpotDetectionPose> &detections);  
  std::shared_ptr<SpotFinderNode> spot_finder_node_;
  rclcpp::Service<SrvTrigger>::SharedPtr srv_restart_patrol_;
  rclcpp::Service<SrvFreeSpots>::SharedPtr srv_finish_patrol_;

  rclcpp::CallbackGroup::SharedPtr cb_group_reeterant_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mutually_exclusive_;

};


}// namespace parkingpatrol

#endif//PARKING_PATROL_PARKING_PATROL_NODE_HPP
