// Copyright 2023 Hotpot DL

#pragma once

#include "armor_tracker/BaseStrategy.hpp"

namespace rm_auto_aim {
    class OmniPercepStrategy : public BaseStrategy
    {
    public:
        OmniPercepStrategy(std::unordered_map<std::string, int>& priority_by_type);
        virtual void init(const std::shared_ptr<Armors>& armors_msg);
        virtual Armor getresultArmor();
        void setTrackedArmor(std::string tracked_id);
        bool switchTO(std::string armor_id);

        std::string tracked_id_;
    };
}