#ifndef PARKING_PATROL_SPOT_FINDER_NODE_HPP
#define PARKING_PATROL_SPOT_FINDER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace parkingpatrol {
using MsgLaserScan = sensor_msgs::msg::LaserScan;
using MsgLaserScan = sensor_msgs::msg::LaserScan;
using MsgPoseWithCovarience = geometry_msgs::msg::PoseWithCovarianceStamped;
using MsgPose = geometry_msgs::msg::Pose;

namespace sectors {
static constexpr char LEFT[] = "LEFT";
static constexpr char RIGHT[] = "RIGHT";
}// namespace sectors

class SpotFinderNode : public rclcpp::Node {

 public:
  SpotFinderNode();

  void clear_poses_buffer();
    
  std::map<std::string, std::vector<MsgPose>> get_poses();
 
 private:
  void declare_parameters();

  void calculate_sector_map();

  void laser_scan_cb(const std::shared_ptr<MsgLaserScan> msg);

  void amcl_pose_cb(const std::shared_ptr<MsgPoseWithCovarience> msg);

  std::vector<float> get_sector_readings(const std::string &sector_name,
                                         const std::vector<float> &lidar_readings);

  // void free_spots_cb(const std::shared_ptr<SrvFreeSpots::Request> req,
  //                    const std::shared_ptr<SrvFreeSpots::Response> res);

  void timer_callback();

  bool detected_free_spot(const std::vector<float> &laser_readings);

  double spot_depth_;
  double spot_width_;
  double lidar_angular_resolution_;
  std::map<std::string, std::array<int, 2>> sector_map_;

  std::vector<MsgPose> free_spots_left_;
  std::vector<MsgPose> free_spots_right_;

  std::mutex pose_mutex_;
  MsgPose current_pose_;
  

  std::mutex laser_mutex_;
  std::vector<float> current_laser_readings_left_;
  std::vector<float> current_laser_readings_right_;

  rclcpp::Subscription<MsgLaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Subscription<MsgPoseWithCovarience>::SharedPtr sub_amcl_pose_;
  // rclcpp::Service<SrvFreeSpots>::SharedPtr srv_free_spots_;

  rclcpp::CallbackGroup::SharedPtr cb_group_reeterant_;
  rclcpp::CallbackGroup::SharedPtr cb_group_mutually_exclusive_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}// namespace parkingpatrol

#endif//PARKING_PATROL_SPOT_FINDER_NODE_HPP
