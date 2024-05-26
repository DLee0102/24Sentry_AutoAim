// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "rm_interfaces/msg/game_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "armor_tracker/trajectory_slover.hpp"
#include "armor_tracker/OmniPercepStrategy.hpp"
namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;
class ArmorTrackerNode : public rclcpp::Node
{
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  inline double deg2rad(double deg) { return deg / 180.0 * M_PI; }
  inline double rad2deg(double rad) { return (rad * 180.0) / M_PI; }

  Eigen::Vector2d calcYawAndPitch(const Eigen::Vector3d & point_cam)
  {
    Eigen::Vector2d offset_angle;
    auto offset_yaw = atan2(point_cam[1], point_cam[0]) ;
    auto offset_pitch =
      -(atan2(point_cam[2], sqrt(point_cam[0] * point_cam[0] + point_cam[1] * point_cam[1])));
    offset_angle << offset_yaw, offset_pitch;
    return offset_angle;
  }
 
  // rcl_interfaces::msg::SetParametersResult parametersCallback(
  //   const std::vector<rclcpp::Parameter> & parameters);
  
  // OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  void declareParameters();

  void initDebugTools();
  
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg, const std::vector<Eigen::Vector3d> & trajectory_msg);

  void setBulletSpeed(const std_msgs::msg::Float64::SharedPtr bulletspeed);

  void setLatancy(const std_msgs::msg::Float64::SharedPtr latency);

  void setAutoAimRestartflag(const std_msgs::msg::Bool::SharedPtr auto_aim_restartflag);

  void createDebugPublishers();
  void destroyDebugPublishers();

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;
  int tracking_thres_;

  // The time when the last message was received
  rclcpp::Time last_time_;
  double dt_;
  double latency_=0.0;
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;

  // Armor tracker
  double lock_thres_;
  double lost_time_thres_;
  std::unordered_map<std::string, int> hit_order_;
  std::unique_ptr<OmniPercepStrategy> omni_strategy_;
  auto_aim_interfaces::msg::Armor fulldetect_armor_;
  auto_aim_interfaces::msg::Armor nav_armor_;
  std::unique_ptr<Tracker> tracker_;

  // OmniPerception
  std::vector<double> pitch_limit_;
  double inside_threshold_yaw_, inside_threshold_pitch_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;
  
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker pred_armor_marker_;
  visualization_msgs::msg::Marker trajectory_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  //trajectory slover param
  int max_iter_;
  float stop_error_;
  int R_K_iter_;
  double init_speed_;
  bool is_hero_;
  //static offset
  double static_offset_yaw_;
  double static_offset_pitch_;


  // outpost param
  double outpost_yaw_angle_thres_;
  double outpost_sqxyz_;
  double outpost_sqyaw_;
  double outpost_sqr_;
  double outpost_r_xyz_factor_;
  double outpost_r_yaw_;
  double outpost_max_match_distance_;
  double outpost_max_match_yaw_diff_;

  // normal param
  double normal_yaw_angle_thres_;
  double normal_sqxyz_;
  double normal_sqyaw_;
  double normal_sqr_;
  double normal_r_xyz_factor_;
  double normal_r_yaw_;
  double normal_max_match_distance_;
  double normal_max_match_yaw_diff_;

  //angle thres
  double yaw_angle_thres_;
  double fire_permit_thres_;
  double fire_latency_;

  // lock yaw thres
  double lock_vyaw_thres_;
  double lock_vyaw_diff_;

  //speed thres
  double min_speed_;

  //car_center_angle
  double car_center_diff_;

  //trajectory_slover
  std::shared_ptr<TrajectorySlover> trajectory_slover_;

  //bullet speed
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bullet_speed_sub_;

  //Subscriber latency
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr latency_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_aim_restartflag_sub_;
  rclcpp::Subscription<rm_interfaces::msg::GameState>::SharedPtr game_state_sub_;

  // Control gimbal by Navigation and OmniPerception
  void navCallback(const auto_aim_interfaces::msg::Armors::SharedPtr nav_msg);
  void omniCallback(const auto_aim_interfaces::msg::Armors::SharedPtr omni_msg);
  void gimbalStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void gameStateCallback(const rm_interfaces::msg::GameState::SharedPtr game_state);

  struct GimbalState
  {
    double yaw;
    double pitch;
  };
  bool nav_control_flag_ = false;
  bool omni_control_flag_ = false;
  GimbalState target_state_;
  GimbalState current_state_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr nav_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr omni_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gimbal_state_sub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
