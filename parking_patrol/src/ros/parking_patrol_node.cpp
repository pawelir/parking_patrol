#include <parking_patrol/ros/parking_patrol_node.hpp>

namespace parkingpatrol
{
ParkingPatrolNode::ParkingPatrolNode(std::shared_ptr<SpotFinderNode> spot_finder_node)
: Node("parking_patrol_node"), spot_finder_node_(std::move(spot_finder_node))
{
  using namespace std::placeholders;

  cb_group_mutually_exclusive_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  srv_restart_patrol_ = create_service<SrvTrigger>(
    "~/restart_patrol", std::bind(&ParkingPatrolNode::restart_patrol_cb_, this, _1, _2),
    rmw_qos_profile_services_default, cb_group_mutually_exclusive_);

  srv_finish_patrol_ = create_service<SrvFreeSpots>(
    "~/finish_patrol", std::bind(&ParkingPatrolNode::finish_patrol_cb_, this, _1, _2),
    rmw_qos_profile_services_default, cb_group_mutually_exclusive_);

  RCLCPP_INFO(get_logger(), "ParkingPatrolNode is initialized");
}

void ParkingPatrolNode::restart_patrol_cb_(
  const std::shared_ptr<SrvTrigger::Request>, const std::shared_ptr<SrvTrigger::Response> res)
{
  RCLCPP_DEBUG(get_logger(), "Processing request for /restart_patrol service");
  try {
    spot_finder_node_->clear_detection_buffer();
    res->success = StatusCode::SUCCEDED;
    res->message = "Restarted patrol";
  } catch (std::runtime_error & error) {
    res->success = StatusCode::REJECTED;
    res->message = error.what();
  }
  RCLCPP_DEBUG(get_logger(), "Finished /restart_patrol service");
}

void ParkingPatrolNode::finish_patrol_cb_(
  const std::shared_ptr<SrvFreeSpots::Request>, const std::shared_ptr<SrvFreeSpots::Response> res)
{
  RCLCPP_DEBUG(get_logger(), "Processing request for /finish_patrol service");
  try {
    auto spot_detections = spot_finder_node_->get_detections();
    auto matched_detections = match_detections_to_columns(spot_detections);
    res->left_column_spots = matched_detections.first;
    res->right_column_spots = matched_detections.second;
    res->result_code = StatusCode::SUCCEDED;
    res->result_string = "Patrol finished correctly";
  } catch (std::runtime_error & error) {
    res->result_code = StatusCode::REJECTED;
    res->result_string = error.what();
  }
  RCLCPP_DEBUG(get_logger(), "Finished /finish_patrol service");
}

std::pair<std::vector<MsgPoint>, std::vector<MsgPoint>>
ParkingPatrolNode::match_detections_to_columns(
  const std::unordered_map<std::string, std::vector<SpotDetectionPose>> & detections)
{
  std::vector<MsgPoint> left_column_spots;
  std::vector<MsgPoint> right_column_spots;

  for (auto & detection : detections.at(side::LEFT)) {
    if (
      abs(abs(detection.theta) - orientation_map_.at(direction::UP)) <=
      orientation_map_.at(direction::TOLERANCE)) {
      left_column_spots.push_back(msg_point_from_detection(detection));
    } else if (
      abs(abs(detection.theta) - orientation_map_.at(direction::DOWN)) <=
      orientation_map_.at(direction::TOLERANCE)) {
      right_column_spots.push_back(msg_point_from_detection(detection));
    }
  }

  for (auto & detection : detections.at(side::RIGHT)) {
    if (
      abs(abs(detection.theta) - orientation_map_.at(direction::UP)) <=
      orientation_map_.at(direction::TOLERANCE)) {
      right_column_spots.push_back(msg_point_from_detection(detection));
    } else if (
      abs(abs(detection.theta) - orientation_map_.at(direction::DOWN)) <=
      orientation_map_.at(direction::TOLERANCE)) {
      left_column_spots.push_back(msg_point_from_detection(detection));
    }
  }
  return std::pair{left_column_spots, right_column_spots};
}

MsgPoint ParkingPatrolNode::msg_point_from_detection(const SpotDetectionPose & detection)
{
  MsgPoint point;
  point.x = detection.x;
  point.y = detection.y;
  return point;
}

}  // namespace parkingpatrol
