#ifndef PARKING_PATROL_SPOT_FINDER_NODE_HPP
#define PARKING_PATROL_SPOT_FINDER_NODE_HPP

#include <algorithm>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <parking_patrol/utils.hpp>

namespace parkingpatrol {
using MsgImu = sensor_msgs::msg::Imu;
using MsgLaserScan = sensor_msgs::msg::LaserScan;
using MsgOdom = nav_msgs::msg::Odometry;
using MsgPose = geometry_msgs::msg::Pose;

class SpotFinderNode : public rclcpp::Node {
public:
  SpotFinderNode();

  void clear_detection_buffer();

  std::map<std::string, std::vector<SpotDetectionPose>> get_detections();

private:
  void declare_parameters();

  void load_parameters();

  void calculate_lidar_range_map();

  void odom_cb(const std::shared_ptr<MsgOdom> msg);

  void laser_scan_cb(const std::shared_ptr<MsgLaserScan> msg);

  std::vector<float> select_readings(const std::string &side_name,
                                     const std::vector<float> &lidar_readings);

  void imu_cb(const std::shared_ptr<MsgImu> msg);

  void timer_callback();

  bool detected_free_spot(const std::vector<float> &laser_readings);

  bool multiple_detection(const std::string &detection_side, float position_x);

  float spot_depth_;
  float spot_width_;
  float lidar_angular_resolution_;

  std::map<std::string, std::array<int, 2>> lidar_range_map_;

  std::vector<SpotDetectionPose> spot_detections_on_left_;
  std::vector<SpotDetectionPose> spot_detections_on_right_;

  std::mutex pose_mutex_;
  MsgPose current_pose_;

  std::mutex laser_mutex_;
  std::vector<float> current_laser_readings_left_;
  std::vector<float> current_laser_readings_right_;

  std::mutex imu_mutex_;
  float current_imu_orientation_;

  rclcpp::Subscription<MsgOdom>::SharedPtr sub_odom_;
  rclcpp::Subscription<MsgLaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Subscription<MsgImu>::SharedPtr sub_imu_;

  rclcpp::CallbackGroup::SharedPtr cb_group_reeterant_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace parkingpatrol

#endif // PARKING_PATROL_SPOT_FINDER_NODE_HPP
