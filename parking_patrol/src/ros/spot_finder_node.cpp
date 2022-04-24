#include "parking_patrol/ros/spot_finder_node.hpp"
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <algorithm>
namespace parkingpatrol {

SpotFinderNode::SpotFinderNode() : Node("spot_finder_node") {
  using namespace std::placeholders;
  declare_parameters();

	this->get_parameter<double>("spot_width", spot_width_);
  this->get_parameter<double>("spot_depth", spot_depth_);
	this->get_parameter<double>("lidar_angular_resolution", lidar_angular_resolution_);

	calculate_sector_map();

  cb_group_mutually_exclusive_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_reeterant_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(
      rmw_qos_profile_sensor_data.history,
      rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data);
  auto default_qos = rclcpp::QoS(rclcpp::QoSInitialization(
      rmw_qos_profile_sensor_data.history,
      rmw_qos_profile_sensor_data.depth), rmw_qos_profile_default);

  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = cb_group_reeterant_;

	sub_amcl_pose_ = this->create_subscription<MsgPoseWithCovarience>(
			"/amcl_pose", default_qos,
      std::bind(&SpotFinderNode::amcl_pose_cb, this, _1),
      sub_options);


  sub_laser_scan_ = this->create_subscription<MsgLaserScan>(
      "/scan", sensor_qos,
      std::bind(&SpotFinderNode::laser_scan_cb, this, _1),
      sub_options);

  // srv_free_spots_ = create_service<SrvFreeSpots>(
	// 		"~/get_free_spots",
	// 		std::bind(&SpotFinderNode::free_spots_cb, this, _1, _2),
	// 		rmw_qos_profile_services_default,
	// 		cb_group_mutually_exclusive_);

	timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
																	 std::bind(&SpotFinderNode::timer_callback,
																	 this));

  RCLCPP_INFO(get_logger(), "SpotFinderNode is initialized");
}

void SpotFinderNode::declare_parameters() {
  this->declare_parameter<double>("spot_width", spot_width_);
  this->declare_parameter<double>("spot_depth", spot_depth_);
	this->declare_parameter<double>("lidar_angular_resolution", lidar_angular_resolution_);
}

void SpotFinderNode::calculate_sector_map() {
	auto sector_size = 360/20;
	int lidar_sector_size = static_cast<int>(sector_size/lidar_angular_resolution_);
	sector_map_= {
		{sectors::LEFT, {lidar_sector_size*4, lidar_sector_size*6}},
		{sectors::RIGHT, {lidar_sector_size*14, lidar_sector_size*16}}
	};
}

void SpotFinderNode::amcl_pose_cb(const MsgPoseWithCovarience::SharedPtr msg) {
  std::lock_guard<std::mutex> lg(pose_mutex_);
	current_pose_ = msg->pose.pose;
}

void SpotFinderNode::laser_scan_cb(const MsgLaserScan::SharedPtr msg) {
	auto current_laser_readings = msg->ranges;
	std::lock_guard<std::mutex> lg(laser_mutex_);
	current_laser_readings_left_ = get_sector_readings(sectors::LEFT, current_laser_readings);
	current_laser_readings_right_ = get_sector_readings(sectors::RIGHT, current_laser_readings);
}

std::vector<float> SpotFinderNode::get_sector_readings(
      const std::string &sector_name,
      const std::vector<float> &lidar_readings) {
	auto sector_readings = std::vector<float>(
        lidar_readings.begin() + sector_map_.at(sector_name).at(0),
        lidar_readings.begin() + sector_map_.at(sector_name).at(1));
	return sector_readings;
}

// void SpotFinderNode::free_spots_cb(const std::shared_ptr<SrvFreeSpots::Request> req,
//                                    const std::shared_ptr<SrvFreeSpots::Response> res) {
//   RCLCPP_DEBUG(get_logger(), "Processing request for /ger_free_spots service");
//   try {
//     res->free_spots = free_spots_;
// 		res->result_code = StatusCode::SUCCEDED;
//   } catch (std::runtime_error &error) {
//     res->result_code = StatusCode::REJECTED;
//     res->result_string = error.what();
//   }
//   RCLCPP_DEBUG(get_logger(), "Finished /get_free_spots service");
// }

void SpotFinderNode::timer_callback() {
	std::vector<float> laser_readings_left;
	std::vector<float> laser_readings_right;
	{
		std::lock_guard<std::mutex> lg(laser_mutex_);
		laser_readings_left = current_laser_readings_left_;
		laser_readings_right = current_laser_readings_right_; 
	}
	if (detected_free_spot(laser_readings_left)) {
		std::lock_guard<std::mutex> lg(pose_mutex_);
		free_spots_left_.emplace_back(current_pose_);
	} if (detected_free_spot(laser_readings_right)) {
		std::lock_guard<std::mutex> lg(pose_mutex_);
		free_spots_right_.emplace_back(current_pose_);
	}
}

bool SpotFinderNode::detected_free_spot(const std::vector<float> &lidar_readings) {
	return *std::min_element(lidar_readings.begin(), 
													 lidar_readings.end()) >= spot_depth_;
}

void SpotFinderNode::clear_poses_buffer() {
	free_spots_left_.clear();
	free_spots_right_.clear();
}

std::map<std::string, std::vector<MsgPose>> SpotFinderNode::get_poses() {
	return std::map<std::string, std::vector<MsgPose>> {
		{sectors::LEFT, free_spots_left_},
		{sectors::RIGHT, free_spots_right_}
	};
}

}// namespace parkingpatrol
