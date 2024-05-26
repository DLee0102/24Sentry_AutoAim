// Copyright 2023 Hotpot DL

#pragma once

// STL
#include <vector>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/armor.hpp"
#include "armor_tracker/TargetInterface.hpp"

namespace rm_auto_aim {
    class Optimization
    {
    public:
        using Armors = auto_aim_interfaces::msg::Armors;
        using Armor = auto_aim_interfaces::msg::Armor;
        Optimization();
        ~Optimization() = default;

        void setCamerainfo(const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs, const std::array<double, 12> & project_matrix);
        std::vector<cv::Point2f> projectYaw(std::vector<cv::Point3f> objectPoints, float yaw, cv::Point3f armor_center, int32_t sec, int32_t nsec);
        void optimizeYaw(Armor& armor);
        void optimizeYaw(ArmorInfo& armor);
        void setTfBuffer(std::shared_ptr<tf2_ros::Buffer> tf2_buffer);
        void setTargetFrame(const std::string & target_frame);

        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        cv::Mat project_matrix_;
        std::string target_frame_;

    private:
        inline double deg2rad(double deg) { return deg / 180.0 * M_PI; }
        inline double rad2deg(double rad) { return (rad * 180.0) / M_PI; }
        double loss_f(const std::vector<cv::Point2f>& contourx, const std::vector<cv::Point2f>& detectedcontour);
        double findMinLoss(const std::vector<cv::Point3f>& Points3d, const std::vector<cv::Point2f>& detectedcontour,
            cv::Point3f armor_center, double origin_yaw, int32_t sec, int32_t nsec);
        double IoU(const std::vector<cv::Point2f>& projectedContour, const std::vector<cv::Point2f>& detectedContour);

        // Unit: mm
        static constexpr float SMALL_ARMOR_WIDTH = 135;
        static constexpr float SMALL_ARMOR_HEIGHT = 55;
        // 从225修改为230
        static constexpr float LARGE_ARMOR_WIDTH = 230;
        static constexpr float LARGE_ARMOR_HEIGHT = 55;
        // Four vertices of armor in 3d
        std::vector<cv::Point3f> small_armor_points_;
        std::vector<cv::Point3f> large_armor_points_;
    };
} // namespace rm_auto_aim