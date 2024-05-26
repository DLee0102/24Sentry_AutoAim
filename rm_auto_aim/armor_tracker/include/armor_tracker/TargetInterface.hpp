// Copyright 2023 Hotpot DL

#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>

namespace rm_auto_aim {
    enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };
    struct ArmorInfo
    {
        std::string number;
        std::string type;
        float distance_to_image_center;
        Eigen::Vector3d position;
        Eigen::Vector4d orientation;
        std::vector<Eigen::Vector2d> img_features;
        std::string frame_id;
        int32_t sec;
        int32_t nsec;
    };
}