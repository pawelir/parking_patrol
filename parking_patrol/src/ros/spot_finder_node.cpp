#include "parking_patrol/ros/spot_finder_node.hpp"
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <algorithm>
namespace parkingpatrol {

SpotFinderNode::SpotFinderNode() : Node("spot_finder_node") {
  using namespace std::placeholders;
  declare_parameters();

  this->get_parameter<float>("spot_depth", spot_depth_);
	this->get_parameter<float>("lidar_angular_resolution", lidar_angular_resolution_);

	calculate_lidar_range_map();

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

	sub_odom_ = this->create_subscription<MsgOdom>(
			"/odom", default_qos,
      std::bind(&SpotFinderNode::odom_cb, this, _1),
      sub_options);

  sub_laser_scan_ = this->create_subscription<MsgLaserScan>(
      "/scan", sensor_qos,
      std::bind(&SpotFinderNode::laser_scan_cb, this, _1),
      sub_options);

  sub_imu_ = this->create_subscription<MsgImu>(
      "/imu", sensor_qos,
      std::bind(&SpotFinderNode::imu_cb, this, _1),
      sub_options);

	timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
																	 std::bind(&SpotFinderNode::timer_callback,
																	 this));

  RCLCPP_INFO(get_logger(), "SpotFinderNode is initialized");
}

void SpotFinderNode::declare_parameters() {
  this->declare_parameter<float>("spot_depth", spot_depth_);
	this->declare_parameter<float>("lidar_angular_resolution", lidar_angular_resolution_);
}

void SpotFinderNode::calculate_lidar_range_map() {
	auto range_size = 360/20;
	int lidar_range_size = static_cast<int>(range_size/lidar_angular_resolution_);
	lidar_range_map_= {
		{side::LEFT, {lidar_range_size*4, lidar_range_size*6}},
		{side::RIGHT, {lidar_range_size*14, lidar_range_size*16}}
	};
}

void SpotFinderNode::odom_cb(const MsgOdom::SharedPtr msg) {
  std::lock_guard<std::mutex> lg(pose_mutex_);
	current_pose_ = msg->pose.pose;
}

void SpotFinderNode::laser_scan_cb(const MsgLaserScan::SharedPtr msg) {
	auto current_laser_readings = msg->ranges;
	std::lock_guard<std::mutex> lg(laser_mutex_);
	current_laser_readings_left_ = select_readings(side::LEFT, current_laser_readings);
	current_laser_readings_right_ = select_readings(side::RIGHT, current_laser_readings);
}

std::vector<float> SpotFinderNode::select_readings(
		const std::string &side_name,
		const std::vector<float> &lidar_readings) {
	auto side_readings = std::vector<float>(
			lidar_readings.begin() + lidar_range_map_.at(side_name).at(0),
			lidar_readings.begin() + lidar_range_map_.at(side_name).at(1));
	return side_readings;
}

void SpotFinderNode::imu_cb(const MsgImu::SharedPtr msg) {
	std::lock_guard<std::mutex> lg(imu_mutex_);
	current_imu_orientation_ = msg->orientation.z;
}

void SpotFinderNode::timer_callback() {
	std::vector<float> laser_readings_left;
	std::vector<float> laser_readings_right;
	{
		std::lock_guard<std::mutex> lg(laser_mutex_);
		laser_readings_left = current_laser_readings_left_;
		laser_readings_right = current_laser_readings_right_; 
	}
	try {
		if (detected_free_spot(laser_readings_left)) {
			std::scoped_lock {pose_mutex_, imu_mutex_};
			spot_detections_on_left_.emplace_back(
					SpotDetectionPose(current_pose_.position.x,
														current_pose_.position.y,
														current_imu_orientation_));
		} if (detected_free_spot(laser_readings_right)) {
			std::scoped_lock {pose_mutex_, imu_mutex_};
			spot_detections_on_right_.emplace_back(
					SpotDetectionPose(current_pose_.position.x,
									current_pose_.position.y,
									current_imu_orientation_));
		}
	} catch(const std::exception& e) {
		RCLCPP_WARN(this->get_logger(), e.what());
	}
}

bool SpotFinderNode::detected_free_spot(const std::vector<float> &lidar_readings) {
	if ( lidar_readings.size() > 0 ) {
		return *std::min_element(lidar_readings.begin(), 
													   lidar_readings.end()) >= spot_depth_;
	} else throw std::invalid_argument( "No LiDAR readings!" );
}

void SpotFinderNode::clear_detection_buffer() {
	spot_detections_on_left_.clear();
	spot_detections_on_right_.clear();
}

std::map<std::string, std::vector<SpotDetectionPose>> SpotFinderNode::get_detections() {
	return std::map<std::string, std::vector<SpotDetectionPose>> {
		{side::LEFT, spot_detections_on_left_},
		{side::RIGHT, spot_detections_on_right_}
	};
}

}// namespace parkingpatrol
