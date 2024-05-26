// Copyright 2023 Hotpot DL

#include "armor_tracker/BaseStrategy.hpp"

namespace rm_auto_aim {

    BaseStrategy::BaseStrategy(std::unordered_map<std::string, int>& priority_by_type)
        : priority_by_type_(priority_by_type)
    {
    }
    BaseStrategy::~BaseStrategy() {}
    Armor BaseStrategy::filtAromorbyDistance()
    {
        double min_distance = 20.0;
        Armor tracked_armor;
        for (const auto& item : armors_msg_vec_)
        {
            double this_distance = Eigen::Vector2d(item.pose.position.x, item.pose.position.y).norm();
            if (this_distance < min_distance)
            {
                min_distance = this_distance;
                tracked_armor = item;
            }
        }

        return tracked_armor;
    }

    void BaseStrategy::filtArmorbyType()
    {
        int prior_min = 100;
        for (const auto& item : armors_msg_vec_)
        {
            auto type_level = priority_by_type_[item.number];
            if (type_level < prior_min)
            {
                prior_min = type_level;
            }
        }
        armors_msg_vec_.erase(
            std::remove_if(
                armors_msg_vec_.begin(), armors_msg_vec_.end(),
                [this, prior_min](const Armor & armor) {
                    return priority_by_type_.find(armor.number)->second > prior_min;
                }),
            armors_msg_vec_.end());
    }
}