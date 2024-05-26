// Copyright 2023 Hotpot DL

#pragma once

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

// STD
#include <memory>
#include <string>
#include <unordered_map>
#include <cfloat>
#include <iostream>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim {
    using Armors = auto_aim_interfaces::msg::Armors;
    using Armor = auto_aim_interfaces::msg::Armor;
    class BaseStrategy
    {
    public:

        BaseStrategy(std::unordered_map<std::string, int>& priority_by_type);
        virtual ~BaseStrategy();
        virtual void init(const std::shared_ptr<Armors>& armors_msg) = 0;
        virtual Armor getresultArmor() = 0;
    
    protected:
        virtual void filtArmorbyType();
        virtual Armor filtAromorbyDistance();
        
        std::unordered_map<std::string, int>& priority_by_type_;
        std::vector<Armor> armors_msg_vec_;
    };
}