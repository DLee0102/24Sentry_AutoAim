// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/buffer.h>

// STD
#include <memory>
#include <string>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "armor_tracker/TrackerStrategy.hpp"

#include "armor_tracker/NormalOberser.hpp"
#include "armor_tracker/OutpostObserver.hpp"
#include "armor_tracker/Optimization.hpp"

namespace rm_auto_aim
{

// enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class Tracker
{
public:
  Tracker(double outpost_max_match_distance, double outpost_max_match_yaw_diff, std::unordered_map<std::string, int> & hit_order,
    double lock_thres, double outpost_s2qxyz, double outpost_s2qyaw, double outpost_s2qr, double outpost_r_xyz_factor,
    double outpost_r_yaw, double outpost_yaw_angle_thres, double normal_max_match_distance, double normal_max_match_yaw_diff,
    double normal_s2qxyz, double normal_s2qyaw, double normal_s2qr, double normal_r_xyz_factor, double normal_r_yaw, double normal_yaw_angle_thres);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armors::SharedPtr & armors_msg);

  void update(const Armors::SharedPtr & armors_msg, double dt);

  void resetState();

  ArmorInfo armormsgToArmorInfo(const Armor & a);
  std::vector<ArmorInfo> armorsmsgToArmorInfoVec(const Armors::SharedPtr & armors_msg);

  // ExtendedKalmanFilter ekf;

  int tracking_thres;
  int lost_thres;
  int switch_thres;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
    SWITCH,
    TEMP_SWITCH,
  } tracker_state;

  std::string tracked_id;
  Armor tracked_armor;
  // ArmorsNum tracked_armors_num;

  double info_position_diff;
  double info_yaw_diff;

  std::shared_ptr<BaseObserver> observer_;
  std::shared_ptr<Optimization> optimize_yaw_;

  // Eigen::VectorXd measurement;

  // Eigen::VectorXd target_state;

  // To store another pair of armors message
  // double dz, another_r;

private:
  void initEKF(const Armor & a);

  void updateArmorsNum(const Armor & a);

  void handleArmorJump(const Armor & a);

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  // double max_match_distance_;
  // double max_match_yaw_diff_;

  std::unordered_map<std::string, int>& priority_by_type_;
  std::unique_ptr<TrackerStrategy> tracker_strategy_;
  std::shared_ptr<NormalObserver> normal_observer_;
  std::shared_ptr<OutpostObserver> outpost_observer_;

  int detect_count_;
  int lost_count_;
  int switch_count_;

  // double last_yaw_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
