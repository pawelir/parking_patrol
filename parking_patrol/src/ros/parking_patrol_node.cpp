#include <parking_patrol/ros/parking_patrol_node.hpp>

namespace parkingpatrol {

ParkingPatrolNode::ParkingPatrolNode(std::shared_ptr<SpotFinderNode> spot_finder_node)
: Node("parking_patrol_node"),
    spot_finder_node_(std::move(spot_finder_node))
{
  using namespace std::placeholders;
  
  declare_parameters();
  load_parameters();

  this->get_parameter<float>("spot_width", spot_width_);

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

void ParkingPatrolNode::declare_parameters() {
  this->declare_parameter<float>("spot_width", spot_width_);
	this->declare_parameter<float>("parking_spot_increment", parking_spot_increment_);
}

void ParkingPatrolNode::load_parameters() {
  this->get_parameter<float>("spot_width", spot_width_);
	this->get_parameter<float>("parking_spot_increment", parking_spot_increment_);
}

void ParkingPatrolNode::restart_patrol_cb_(
    const std::shared_ptr<SrvTrigger::Request>,
    const std::shared_ptr<SrvTrigger::Response> res) {
  RCLCPP_DEBUG(get_logger(), "Processing request for /restart_patrol service");
  try {
    spot_finder_node_->clear_detection_buffer();
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
    auto spot_detections = spot_finder_node_->get_detections(); 
    filter_multiple_detections(spot_detections.at(side::LEFT));
    filter_multiple_detections(spot_detections.at(side::RIGHT));
    auto matched_detections = match_detections_to_columns(spot_detections);
    res->left_column_spots = matched_detections.first;
    res->right_column_spots = matched_detections.second;
		res->result_code = StatusCode::SUCCEDED;
  } catch (std::runtime_error &error) {
    res->result_code = StatusCode::REJECTED;
    res->result_string = error.what();
  }
  RCLCPP_DEBUG(get_logger(), "Finished /finish_patrol service");
}

std::vector<SpotDetectionPose> ParkingPatrolNode::filter_multiple_detections(std::vector<SpotDetectionPose> &detections) {
  std::vector<SpotDetectionPose> filtered_detections;
  std::optional<SpotDetectionPose> last_detection;

  for (auto detection : detections) {
    if ( !last_detection || abs(detection.x - last_detection.value().x) <= spot_width_ ) {
      filtered_detections.push_back(detection);
    } else {
      filtered_detections.push_back(detection);
    } last_detection = detection;
  return filtered_detections;
  }
}

std::pair<std::vector<MsgPoint>, std::vector<MsgPoint>> ParkingPatrolNode::match_detections_to_columns(
    const std::map<std::string, std::vector<SpotDetectionPose>> &detections) {
  std::cout << "on matching detections" <<std::endl;
  std::vector<MsgPoint> left_column_spots;
  std::vector<MsgPoint> right_column_spots;


  for (auto detection : detections.at(side::LEFT)) {
    if (abs(detection.theta - orientation_map_.at(UPSIDE)) <= orientation_map_.at(TOLERANCE)) {
      left_column_spots.push_back(msg_point_from_detection(detection));
    } if (abs(detection.theta - orientation_map_.at(DOWNSIDE)) <= orientation_map_.at(TOLERANCE)) {
      right_column_spots.push_back(msg_point_from_detection(detection));
    }
  }

  for (auto detection : detections.at(side::RIGHT)) {
    if (abs(detection.theta - orientation_map_.at(UPSIDE)) <= orientation_map_.at(TOLERANCE)) {
      right_column_spots.push_back(msg_point_from_detection(detection));
    } if (abs(detection.theta - orientation_map_.at(DOWNSIDE)) <= orientation_map_.at(TOLERANCE)) {
      left_column_spots.push_back(msg_point_from_detection(detection));
    }
  }
  return std::pair{left_column_spots, right_column_spots};
}

MsgPoint ParkingPatrolNode::msg_point_from_detection(const SpotDetectionPose &detection) {
  MsgPoint point;
  point.x = detection.x;
  point.y = detection.y;
  return point;
}


}// namespace parkingpatrol
