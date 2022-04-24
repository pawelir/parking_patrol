#include <parking_patrol/ros/parking_patrol_node.hpp>

namespace parkingpatrol {

ParkingPatrolNode::ParkingPatrolNode(std::shared_ptr<SpotFinderNode> spot_finder_node)
  : Node("parking_patrol_node"),
    spot_finder_node_(std::move(spot_finder_node))
{
 using namespace std::placeholders;

  cb_group_mutually_exclusive_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
  srv_restart_patrol_ = create_service<SrvTrigger>(
			"~/restart_patrol",
			std::bind(&ParkingPatrolNode::restart_patrol_cb_, this, _1, _2),
			rmw_qos_profile_services_default,
			cb_group_mutually_exclusive_);
  
  srv_finish_patrol_ = create_service<SrvFreeSpots>(
			"~/finish_patrol",
			std::bind(&ParkingPatrolNode::finish_patrol_cb_, this, _1, _2),
			rmw_qos_profile_services_default,
			cb_group_mutually_exclusive_);

  RCLCPP_INFO(get_logger(), "ParkingPatrolNode is initialized");
}

void ParkingPatrolNode::restart_patrol_cb_(
    const std::shared_ptr<SrvTrigger::Request>,
    const std::shared_ptr<SrvTrigger::Response> res) {
  RCLCPP_DEBUG(get_logger(), "Processing request for /restart_patrol service");
  try {
    spot_finder_node_->clear_poses_buffer();
    res->success = StatusCode::SUCCEDED;
		res->message = "Restarted patrol";
  } catch (std::runtime_error &error) {
    res->success = StatusCode::REJECTED;
    res->message = error.what();
  }
  RCLCPP_DEBUG(get_logger(), "Finished /restart_patrol service");
}

void ParkingPatrolNode::finish_patrol_cb_(
    const std::shared_ptr<SrvFreeSpots::Request>,
    const std::shared_ptr<SrvFreeSpots::Response> res) {
  RCLCPP_DEBUG(get_logger(), "Processing request for /finish_patrol service");
  try {
    auto detected_poses = spot_finder_node_->get_poses(); 
    res->left_side_spots = filter_multiple_poses(detected_poses.at(sectors::LEFT));
    res->right_side_spots = filter_multiple_poses(detected_poses.at(sectors::RIGHT));
		res->result_code = StatusCode::SUCCEDED;
  } catch (std::runtime_error &error) {
    res->result_code = StatusCode::REJECTED;
    res->result_string = error.what();
  }
  RCLCPP_DEBUG(get_logger(), "Finished /finish_patrol service");
}

std::vector<MsgPose> ParkingPatrolNode::filter_multiple_poses(const std::vector<MsgPose> &poses) {
  std::vector<MsgPose> filtered_multiple_detections;
  for (auto pose : poses) {

  }
}

}// namespace parkingpatrol
