// Copyright 2023 Hotpot DL

#include "armor_tracker/OmniPercepStrategy.hpp"

namespace rm_auto_aim {
    using Armors = auto_aim_interfaces::msg::Armors;
    using Armor = auto_aim_interfaces::msg::Armor;
    /* 引用类型的成员需在参数表中进行初始化 */
    OmniPercepStrategy::OmniPercepStrategy(std::unordered_map<std::string, int>& priority_by_type)
        : BaseStrategy(priority_by_type)
    {
    }

    void OmniPercepStrategy::setTrackedArmor(std::string tracked_id)
    {
        tracked_id_ = tracked_id;
    }

    void OmniPercepStrategy::init(const std::shared_ptr<Armors>& armors_msg)
    {
        std::vector<Armor>().swap(armors_msg_vec_);
        for (const auto& item : armors_msg->armors)
        {
            armors_msg_vec_.push_back(item);
        }
    }

    Armor OmniPercepStrategy::getresultArmor()
    {
        Armor tracked_armor;

        filtArmorbyType();
        // 此处不能考虑距离，因为距离解算不精确
        // tracked_armor = filtAromorbyDistance();
        tracked_armor = armors_msg_vec_[0];

        return tracked_armor;
    }

    bool OmniPercepStrategy::switchTO(std::string armor_id)
    {
        return priority_by_type_[armor_id] < priority_by_type_[tracked_id_];
    }
}
