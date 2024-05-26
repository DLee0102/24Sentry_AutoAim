// Copyright 2023 Hotpot DL

#include "armor_tracker/BaseObserver.hpp"

namespace rm_auto_aim {
    BaseObserver::BaseObserver() {}
    BaseObserver::~BaseObserver() {}
    double BaseObserver::orientationToYaw(const Eigen::Vector4d& q)
    {
        // Convert Eigen::Vector4d to tf2::Quaternion
        tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());

        // Get armor yaw
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

        // Make yaw change continuous (-pi~pi to -inf~inf)
        yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
        last_yaw_ = yaw;

        return yaw;
    }

    Eigen::Vector2d BaseObserver::calcYawAndPitch(const Eigen::Vector3d & target_point)
    {
        Eigen::Vector2d offset_angle;
        auto offset_yaw = atan2(target_point[1], target_point[0]) ;
        auto offset_pitch =
        -(atan2(target_point[2], sqrt(target_point[0] * target_point[0] + target_point[1] * target_point[1])));
        offset_angle << offset_yaw, offset_pitch;
        return offset_angle;
    }
}