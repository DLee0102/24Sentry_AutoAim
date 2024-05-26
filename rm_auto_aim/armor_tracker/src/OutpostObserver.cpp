// Copyright 2023 Hotpot DL

#include "armor_tracker/OutpostObserver.hpp"

namespace rm_auto_aim {
    OutpostObserver::OutpostObserver(double s2qxyz, double s2qyaw, double s2qr, 
            double r_xyz_factor, double r_yaw, double max_match_distance, double max_match_yaw_diff, double yaw_angle_thres)
        : s2qxyz_(s2qxyz), s2qyaw_(s2qyaw), s2qr_(s2qr), r_xyz_factor_(r_xyz_factor), r_yaw_(r_yaw)
    {
        // EKF
        // xa = x_armor, xc = x_robot_center
        // state: xc, yc, za , yaw, v_yaw, r
        // measurement: xa, ya, za, yaw
        // f - Process function
        auto f = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(3) += x(4) * dt_; // yaw = yaw + v_yaw * dt  
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_f = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(6, 6);
            // clang-format off
                //xc yc zc yaw v_yaw r
            f <<  1, 0, 0, 0, 0,  0,
                0, 1, 0, 0, 0,  0, 
                0, 0, 1, 0, 0,  0,
                0, 0, 0, 1, dt_,0,
                0, 0, 0, 0, 1,  0,
                0, 0, 0, 0, 0,  1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto h = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(1), yaw = x(3), r = x(5);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(2);               // za
            z(3) = x(3);               // yaw
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_h = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 6);
            double yaw = x(3), r = x(5);
            // clang-format off
            //    xc  yc  za  yaw         v_yaw r
            h <<  1,  0,  0,  r*sin(yaw), 0,   -cos(yaw),
                0,  1,  0, -r*cos(yaw), 0,   -sin(yaw),
                0,  0,  1,  0,          0,   0,
                0,  0,  0,  1,          0,   0;
            // clang-format on
            return h;
        };

        // update_Q - process noise covariance matrix
        auto u_q = [this]() {
            Eigen::MatrixXd q(6, 6);
            double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
            double q_x_x = pow(t, 4) / 4 * x;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc      yc     za      yaw     v_yaw   r
            q <<  q_x_x,  0,     0,       0,      0,      0,
                0,      q_x_x, 0,       0,      0,      0,
                0,      0,     q_x_x,   0,      0,      0,
                0,      0,     0,     q_y_y,  q_y_vy,   0,
                0,      0,     0,     q_y_vy, q_vy_vy,  0,
                0,      0,     0,     0,      0,       q_r;
            // clang-format on
            return q;
        };

        // update_R - measurement noise covariance matrix
        auto u_r = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor_;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw_;
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 6> p0;
        p0.setIdentity();
        this->ekf_ = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

        measurement_ = Eigen::VectorXd::Zero(4);
        target_state_ = Eigen::VectorXd::Zero(6);
        max_match_distance_ = max_match_distance;
        max_match_yaw_diff_ = max_match_yaw_diff;
        yaw_angle_thres_ = yaw_angle_thres;
        tracked_id_ = std::string("");
    }

    OutpostObserver::~OutpostObserver() {}

    Eigen::Vector3d OutpostObserver::getArmorPositionFromState(const Eigen::VectorXd & x)
    {
        // Calculate predicted position of the current armor
        double xc = x(0), yc = x(1), za = x(2);
        double yaw = x(3), r = x(5);
        double xa = xc - r * cos(yaw);
        double ya = yc - r * sin(yaw);
        return Eigen::Vector3d(xa, ya, za);
    }

    void OutpostObserver::initFilter(const ArmorInfo & a)
    {
        RCLCPP_INFO(rclcpp::get_logger("armor_tracker"), "Init outpost observer");
        double xa = a.position.x();
        double ya = a.position.y();
        double za = a.position.z();
        last_yaw_ = 0;
        double yaw = orientationToYaw(a.orientation);

        // Set initial position at 0.2m behind the target
        target_state_ = Eigen::VectorXd::Zero(6);
        double r = 0.26;
        double xc = xa + r * cos(yaw);
        double yc = ya + r * sin(yaw);
        dz_ = 0, another_r_ = r;
        target_state_ << xc, yc, za, yaw, 0, r;

        ekf_.setState(target_state_);
        tracked_id_ = a.number;
        tracked_armor_ = a;
        tracked_armors_num_ = ArmorsNum::OUTPOST_3;
    }

    void OutpostObserver::updateFilter(std::vector<ArmorInfo> armors, double dt, bool& matched)
    {
        dt_ = dt;
        Eigen::VectorXd ekf_prediction = ekf_.predict();
        matched = false;
        // Use KF prediction as default target state if no matched armor is found
        target_state_ = ekf_prediction;

        if (!armors.empty())
        {
            // Find the closest armor with the same id
            ArmorInfo same_id_armor;
            int same_id_armors_count = 0;
            auto predicted_position = getArmorPositionFromState(ekf_prediction);
            double min_position_diff = DBL_MAX;
            double yaw_diff = DBL_MAX;
            for (const auto & armor : armors)
            {
                // Only consider armors with the same id
                if (armor.number == tracked_id_)
                {
                    same_id_armor = armor;
                    same_id_armors_count++;
                    // Calculate the difference between the predicted position and the current armor position
                    Eigen::Vector3d position_vec = armor.position;
                    double position_diff = (predicted_position - position_vec).norm();
                    if (position_diff < min_position_diff)
                    {
                        // Find the closest armor
                        min_position_diff = position_diff;
                        yaw_diff = abs(orientationToYaw(armor.orientation) - ekf_prediction(3));
                        tracked_armor_ = armor;
                    }
                }
            }

            info_position_diff_ = min_position_diff;
            info_yaw_diff_ = yaw_diff;

            // optimize_yaw_->optimizeYaw(tracked_armor_);

            // Check if the distance and yaw difference of closest armor are within the threshold
            if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_)
            {
                // Matched armor found
                matched = true;
                auto p = tracked_armor_.position;
                // Update EKF
                double measured_yaw = orientationToYaw(tracked_armor_.orientation);
                measurement_ = Eigen::Vector4d(p.x(), p.y(), p.z(), measured_yaw);
                target_state_ = ekf_.update(measurement_);

                RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update");
            }
            else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_)
            {
                // Matched armor not found, but there is only one armor with the same id
                // and yaw has jumped, take this case as the target is spinning and armor jumped
                handleArmorJump(same_id_armor);
            }
            else
            {
                // No matched armor found
                RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found!");
            }
        }

        target_state_(5) = 0.25685;
        if (target_state_(4) > 1.5) target_state_(4) = 2.4;
    }

    void OutpostObserver::handleArmorJump(const ArmorInfo& current_armor)
    {
        double yaw = orientationToYaw(current_armor.orientation);
        target_state_(3) = yaw;
        tracked_armors_num_ = ArmorsNum::OUTPOST_3;
        // Only 4 armors has 2 radius and height

        RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

        // If position difference is larger than max_match_distance_,
        // take this case as the ekf diverged, reset the state
        auto p = current_armor.position;
        Eigen::Vector3d current_p(p.x(), p.y(), p.z());
        Eigen::Vector3d infer_p = getArmorPositionFromState(target_state_);
        if ((current_p - infer_p).norm() > max_match_distance_) {
            double r = target_state_(5);
            target_state_(0) = p.x() + r * cos(yaw);  // xc
            target_state_(1) = p.y() + r * sin(yaw);  // yc
            target_state_(2) = p.z();                 // za
            RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Reset State!");
        }

        ekf_.setState(target_state_);
    }

    void OutpostObserver::getTargetArmorPosition(double pred_dt, Eigen::Vector3d& armor_target_min_yaw_diff, double & min_yaw_diff, bool& tracked_permit)
    {
        Eigen::Vector3d now_car_pos = Eigen::Vector3d(target_state_(0), target_state_(1), target_state_(2));
        double armor_yaw = target_state_(3);
        double car_w = target_state_(4);
        double armor_r = target_state_(5);

        Eigen::Vector3d pred_car_pos = now_car_pos;
        double pred_yaw = armor_yaw + car_w * pred_dt;
        double car_center_diff = calcYawAndPitch(pred_car_pos)[0];

        Eigen::Vector3d pred_armor_pos;

        size_t a_n = static_cast<int>(tracked_armors_num_);
        double r = 0.25685;
        std::vector<std::pair<Eigen::Vector3d, double>> temp_armors_pos_vec;
        for (size_t i = 0; i < a_n; i++) {
            double tmp_yaw = pred_yaw + i * (2 * M_PI / a_n);

            r = armor_r; 
            pred_armor_pos[2] = pred_car_pos[2];
            pred_armor_pos[0] = pred_car_pos[0] - r * cos(tmp_yaw);
            pred_armor_pos[1] = pred_car_pos[1] - r * sin(tmp_yaw);

            temp_armors_pos_vec.emplace_back(std::make_pair(pred_armor_pos, tmp_yaw));

            //get armor yaw  
            double a_yaw = tmp_yaw;
            if (a_yaw >= M_PI)
            {
                a_yaw -= 2.0*M_PI ;
            }

            auto tmp_yaw_diff = fabs(a_yaw - car_center_diff);
            if (tmp_yaw_diff >= M_PI)
            {
                tmp_yaw_diff -= 2.0*M_PI ;
            }
            // std::cout << "yaw: " << rad2deg(tmp_yaw_diff) << " " << rad2deg(car_center_diff) << " ";
            if(fabs(tmp_yaw_diff) < min_yaw_diff)
            {
                min_yaw_diff = fabs(tmp_yaw_diff);
                armor_target_min_yaw_diff = pred_armor_pos;
                target_armor_pos_.second = tmp_yaw;
            }
        }
        // std::cout << std::endl << "final yaw: " << rad2deg(min_yaw_diff) << std::endl;
        if (rad2deg(min_yaw_diff) < yaw_angle_thres_ )
            tracked_permit = 1;
        target_armors_pos_vec_ = temp_armors_pos_vec;
        target_armor_pos_.first = armor_target_min_yaw_diff;
    }
}