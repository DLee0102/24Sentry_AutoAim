// Copyright 2022 Chen Jun

#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>
#include <iostream>

namespace rm_auto_aim
{
Tracker::Tracker(double outpost_max_match_distance, double outpost_max_match_yaw_diff, std::unordered_map<std::string, int> & hit_order,
    double lock_thres, double outpost_s2qxyz, double outpost_s2qyaw, double outpost_s2qr, double outpost_r_xyz_factor,
    double outpost_r_yaw, double outpost_yaw_angle_thres, double normal_max_match_distance, double normal_max_match_yaw_diff,
    double normal_s2qxyz, double normal_s2qyaw, double normal_s2qr, double normal_r_xyz_factor, double normal_r_yaw, double normal_yaw_angle_thres)
: tracker_state(LOST),
  tracked_id(std::string("")),
  // measurement(Eigen::VectorXd::Zero(4)),
  // target_state(Eigen::VectorXd::Zero(9)),
  // max_match_distance_(max_match_distance),
  // max_match_yaw_diff_(max_match_yaw_diff),
  priority_by_type_(hit_order),
  detect_count_(0),
  lost_count_(0),
  switch_count_(0)
{
  tracker_strategy_ = std::make_unique<TrackerStrategy>(priority_by_type_, lock_thres);
  normal_observer_ = std::make_shared<NormalObserver>(normal_s2qxyz, normal_s2qyaw, normal_s2qr, normal_r_xyz_factor, normal_r_yaw, normal_max_match_distance, normal_max_match_yaw_diff, normal_yaw_angle_thres);
  outpost_observer_ = std::make_shared<OutpostObserver>(outpost_s2qxyz, outpost_s2qyaw, outpost_s2qr, outpost_r_xyz_factor, outpost_r_yaw, outpost_max_match_distance, outpost_max_match_yaw_diff, outpost_yaw_angle_thres);
  optimize_yaw_ = std::make_shared<Optimization>();
  normal_observer_->optimize_yaw_ = optimize_yaw_;
  outpost_observer_->optimize_yaw_ = optimize_yaw_;
}

ArmorInfo Tracker::armormsgToArmorInfo(const Armor & a)
{
  ArmorInfo armor_info;
  armor_info.number = a.number;
  armor_info.type = a.type;
  armor_info.orientation = Eigen::Vector4d(a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w);
  armor_info.position = Eigen::Vector3d(a.pose.position.x, a.pose.position.y, a.pose.position.z);
  armor_info.distance_to_image_center = a.distance_to_image_center;
  for (auto feature_p : a.img_features)
  {
    armor_info.img_features.emplace_back(Eigen::Vector2d(feature_p.x, feature_p.y));
  }
  armor_info.frame_id = a.header.frame_id;
  armor_info.sec = a.header.stamp.sec;
  armor_info.nsec = a.header.stamp.nanosec;

  return armor_info;
}

std::vector<ArmorInfo> Tracker::armorsmsgToArmorInfoVec(const Armors::SharedPtr & armors_msg)
{
  std::vector<ArmorInfo> armor_info_vec;
  for (const auto & armor : armors_msg->armors)
  {
    armor_info_vec.push_back(armormsgToArmorInfo(armor));
  }

  return armor_info_vec;
}

void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    return;
  }

  /* 处理击打时间，LOST状态下要同时初始化两个时间，而SWITCH状态下只需要更新一个时间 */
  if (this->tracker_state == SWITCH)
  {
    tracker_strategy_->updateTracktime(armors_msg->header.stamp);
  }
  else
  {
    tracker_strategy_->initTracktime(armors_msg->header.stamp);
  }

  tracker_strategy_->init(armors_msg);
  
  tracked_armor = tracker_strategy_->getresultArmor();
  // optimize_yaw_->optimizeYaw(tracked_armor);

  // initEKF(tracked_armor);
  if (tracked_armor.number != "outpost")
  {
    normal_observer_->initFilter(armormsgToArmorInfo(tracked_armor));
    observer_ = normal_observer_;
  }
  else
  {
    outpost_observer_->initFilter(armormsgToArmorInfo(tracked_armor));
    observer_ = outpost_observer_;
  }
  
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;

  // updateArmorsNum(tracked_armor);
}

void Tracker::update(const Armors::SharedPtr & armors_msg, double dt)
{
    bool matched = false;
    bool switched = false;

    observer_->updateFilter(armorsmsgToArmorInfoVec(armors_msg), dt, matched);

    for (const auto & armor : armors_msg->armors)
    {
      if (armor.number != tracked_id)
      {
        int this_type = priority_by_type_[armor.number];
        int tracker_type = priority_by_type_[tracked_id];

        double this_distance_to_gimbal = Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm();
        double tracker_distance_to_gimbal =
          Eigen::Vector2d(observer_->tracked_armor_.position.x(), observer_->tracked_armor_.position.y()).norm();

        if (this_type < tracker_type || this_distance_to_gimbal < tracker_distance_to_gimbal)
        {
          switched = true;
        }
      }
    }

    // Store tracker info
    info_position_diff = observer_->info_position_diff_;
    info_yaw_diff = observer_->info_yaw_diff_;

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
    if (switched) {
      tracker_state = TEMP_SWITCH;
      switch_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > std::abs(lost_thres)) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  } else if (tracker_state == TEMP_SWITCH) {
    if (switched) {
      switch_count_++;
      if (switch_count_ > switch_thres) {
        switch_count_ = 0;
        tracker_state = SWITCH;
      }
    } else {
      tracker_state = TRACKING;
      switch_count_ = 0;
    }
  }
}

/* 重置跟踪器状态 */
void Tracker::resetState()
{
  tracker_state = LOST;
  detect_count_ = 0;
  switch_count_ = 0;
  lost_count_ = 0;
}

}  // namespace rm_auto_aim
