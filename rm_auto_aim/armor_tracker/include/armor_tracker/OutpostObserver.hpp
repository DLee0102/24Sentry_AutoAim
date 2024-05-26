// Copyright 2023 Hotpot DL

#pragma once

#include "armor_tracker/extended_kalman_filter.hpp"
#include "armor_tracker/BaseObserver.hpp"
#include "armor_tracker/Optimization.hpp"
#include <functional>

namespace rm_auto_aim {
    class OutpostObserver : public BaseObserver
    {
    public:
        OutpostObserver(double s2qxyz, double s2qyaw, double s2qr, 
            double r_xyz_factor, double r_yaw, double max_match_distance, double max_match_yaw_diff, double yaw_angle_thres);
        virtual ~OutpostObserver();

        virtual void initFilter(const ArmorInfo & a) override;
        virtual void updateFilter(std::vector<ArmorInfo> armors, double dt, bool & matched) override;
        virtual void handleArmorJump(const ArmorInfo & current_armor) override;
        virtual Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x) override;
        // virtual void updateArmorsNum(const ArmorInfo & armor);
        virtual void getTargetArmorPosition(double pred_dt, Eigen::Vector3d & armor_target_min_yaw_diff, double& min_yaw_diff, bool & tracked_permit) override;

        double info_position_diff_;
        double info_yaw_diff_;
        double max_match_distance_;
        double max_match_yaw_diff_;
        std::shared_ptr<Optimization> optimize_yaw_;

    private:
        double s2qxyz_, s2qyaw_, s2qr_;
        double r_xyz_factor_, r_yaw_;
    };
}