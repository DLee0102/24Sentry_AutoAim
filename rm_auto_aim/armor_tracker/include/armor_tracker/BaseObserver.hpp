// Copyright 2023 Hotpot DL

#pragma once

#include "armor_tracker/extended_kalman_filter.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>

#include <string>
#include <vector>

#include "armor_tracker/TargetInterface.hpp"

namespace rm_auto_aim {
    class BaseObserver
    {
    public:
        BaseObserver();
        virtual ~BaseObserver();

        virtual void initFilter(const ArmorInfo& a) = 0;
        virtual void updateFilter(std::vector<ArmorInfo> armors, double dt, bool& matched) = 0;
        virtual void handleArmorJump(const ArmorInfo& current_armor) = 0;
        virtual double orientationToYaw(const Eigen::Vector4d& q);
        virtual Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x) = 0;
        virtual void getTargetArmorPosition(double pred_dt, Eigen::Vector3d& armor_target_min_yaw_diff, double & min_yaw_diff, bool & tracked_permit) = 0;
        virtual Eigen::Vector2d calcYawAndPitch(const Eigen::Vector3d & target_point);

        ExtendedKalmanFilter ekf_;
        Eigen::VectorXd measurement_;
        Eigen::VectorXd target_state_;
        ArmorInfo tracked_armor_;
        ArmorsNum tracked_armors_num_;
        std::string tracked_id_;

        std::vector<std::pair<Eigen::Vector3d, double>> target_armors_pos_vec_;
        std::pair<Eigen::Vector3d, double> target_armor_pos_;

        double dt_;
        double dz_;
        double another_r_;
        double info_position_diff_;
        double info_yaw_diff_;
        
        // 存储上一次yaw的值，用于让yaw在(-pi~pi to -inf~inf)范围内连续变化
        double last_yaw_;
    protected:
        inline double deg2rad(double deg) { return deg / 180.0 * M_PI; }
        inline double rad2deg(double rad) { return (rad * 180.0) / M_PI; }
        double yaw_angle_thres_;
    };
}