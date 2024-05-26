// Copyright 2023 Hotpot DL
#pragma once

#include "armor_tracker/BaseStrategy.hpp"

namespace rm_auto_aim {
    class TrackerStrategy : public BaseStrategy
    {
    public:
        TrackerStrategy(std::unordered_map<std::string, int>& priority_by_type, double lock_thres);
        virtual void init(const std::shared_ptr<Armors>& armors_msg);
        virtual Armor getresultArmor();
        
        void updateTracktime(rclcpp::Time now_time);
        void initTracktime(rclcpp::Time now_time);
    
    private:
        rclcpp::Time now_track_time_;
        rclcpp::Time last_track_time_;
        double lock_thres_;
    };
}