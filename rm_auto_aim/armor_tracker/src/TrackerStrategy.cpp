// Copyright 2023 Hotpot DL

#include "armor_tracker/TrackerStrategy.hpp"

namespace rm_auto_aim {
    using Armors = auto_aim_interfaces::msg::Armors;
    using Armor = auto_aim_interfaces::msg::Armor;
    /* 引用类型的成员需在参数表中进行初始化 */
    TrackerStrategy::TrackerStrategy(std::unordered_map<std::string, int>& priority_by_type, double lock_thres)
        : BaseStrategy(priority_by_type),
        lock_thres_(lock_thres)
    {
        
    }

    void TrackerStrategy::init(const std::shared_ptr<Armors>& armors_msg)
    {
        std::vector<Armor>().swap(armors_msg_vec_);
        for (const auto& item : armors_msg->armors)
        {
            armors_msg_vec_.push_back(item);
        }
    }

    Armor TrackerStrategy::getresultArmor()
    {
        double min_distance = DBL_MAX;
        Armor tracked_armor;

        /* 当锁定的时间超过阈值时，则说明击打该目标已消耗较大代价，应继续追击该目标，因此此时就改为锁定距图像中心最近的装甲板 */
        if ((now_track_time_ - last_track_time_).seconds() < lock_thres_)
        {
            filtArmorbyType();
            tracked_armor = filtAromorbyDistance();
        }
        else
        {

            tracked_armor = armors_msg_vec_[0];
            for (const auto & armor : armors_msg_vec_) {
                if (armor.distance_to_image_center < min_distance) {
                    min_distance = armor.distance_to_image_center;
                    tracked_armor = armor;
                }
            }

            // last_track_time_ = now_track_time_;
        }

        

        return tracked_armor;
    }

    void TrackerStrategy::updateTracktime(rclcpp::Time now_time)
    {
        now_track_time_ = now_time;
    }

    void TrackerStrategy::initTracktime(rclcpp::Time now_time)
    {
        now_track_time_ = now_time;
        last_track_time_ = now_time;
    }
}