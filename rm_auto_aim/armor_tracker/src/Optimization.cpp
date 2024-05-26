// Copyright 2023 Hotpot DL

#include "armor_tracker/Optimization.hpp"

namespace rm_auto_aim {
    Optimization::Optimization()
    {
        // Unit: m
        constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
        constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
        constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
        constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

        // Start from bottom left in clockwise order
        // Model coordinate: x forward, y left, z up
        small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
        small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
        small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
        small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

        large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
        large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
        large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
        large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
    }

    void Optimization::setCamerainfo(const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs, const std::array<double, 12> & project_matrix)
    {
        camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone();
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone();
        project_matrix_ = cv::Mat(3, 4, CV_64F, const_cast<double *>(project_matrix.data())).clone();
    }

    void Optimization::setTfBuffer(std::shared_ptr<tf2_ros::Buffer> tf2_buffer)
    {
        tf2_buffer_ = tf2_buffer;
    }

    void Optimization::setTargetFrame(const std::string & target_frame)
    {
        target_frame_ = target_frame;
    }

    void Optimization::optimizeYaw(Armor& armor)
    {
        auto object_points = armor.type == "small" ? small_armor_points_ : large_armor_points_;
        geometry_msgs::msg::Quaternion q = armor.pose.orientation;
        tf2::Quaternion tf_q;
        tf2::fromMsg(q, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        yaw = rad2deg(yaw);

        std::vector<cv::Point2f> contour;
        for (int i = 0; i < 4; i++)
        {
            cv::Point2f p;
            p.x = armor.img_features[i].x;
            p.y = armor.img_features[i].y;
            contour.push_back(p);
        }

        cv::Point3f armor_center(armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
        // std::cout << "before armor_center: " << armor_center << std::endl;

        // std::cout << "Initial yaw: " << yaw << std::endl;
        yaw = findMinLoss(object_points, contour, armor_center, yaw, armor.header.stamp.sec, armor.header.stamp.nanosec);
        // std::cout << "Optimized yaw: " << yaw << std::endl;

        tf2::Quaternion q_new;
        q_new.setRPY(0, -0.26, deg2rad(yaw));
        armor.pose.orientation = tf2::toMsg(q_new);
    }

    void Optimization::optimizeYaw(ArmorInfo& armor)
    {
        auto object_points = armor.type == "small" ? small_armor_points_ : large_armor_points_;
        geometry_msgs::msg::Quaternion q = tf2::toMsg(tf2::Quaternion(armor.orientation.x(), armor.orientation.y(), armor.orientation.z(), armor.orientation.w()));
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
        yaw = rad2deg(yaw);

        std::vector<cv::Point2f> contour;
        for (int i = 0; i < 4; i++)
        {
            cv::Point2f p;
            p.x = armor.img_features[i][0];
            p.y = armor.img_features[i][1];
            contour.push_back(p);
        }

        cv::Point3f armor_center(armor.position.x(), armor.position.y(), armor.position.z());
        // std::cout << "before armor_center: " << armor_center << std::endl;

        // std::cout << "Initial yaw: " << yaw << std::endl;
        yaw = findMinLoss(object_points, contour, armor_center, yaw, armor.sec, armor.nsec);
        // std::cout << "Optimized yaw: " << yaw << std::endl;

        tf2::Quaternion q_new;
        q_new.setRPY(0, -0.26, deg2rad(yaw));
        armor.orientation = {q_new.x(), q_new.y(), q_new.z(), q_new.w()};
    }

    double Optimization::findMinLoss(const std::vector<cv::Point3f>& Points3d, const std::vector<cv::Point2f>& detectedcontour,
        cv::Point3f armor_center, double origin_yaw, int32_t sec, int32_t nsec)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        int iter_count = 0;

        std::unordered_map<int, std::pair<int, int>> fibonacciDict;
        fibonacciDict[2] = std::make_pair(1, 1);
        fibonacciDict[3] = std::make_pair(1, 2);
        fibonacciDict[5] = std::make_pair(2, 3);
        fibonacciDict[8] = std::make_pair(3, 5);
        fibonacciDict[13] = std::make_pair(5, 8);
        fibonacciDict[21] = std::make_pair(8, 13);
        fibonacciDict[34] = std::make_pair(13, 21);
        fibonacciDict[55] = std::make_pair(21, 34);

        int left = 0, right = 55;
        double start = origin_yaw - 25.5;
        int mid1 = left + fibonacciDict[right - left].first;
        int mid2 = left + fibonacciDict[right - left].second;
        double yaw1 = start + mid1;
        double yaw2 = start + mid2;
        std::vector<cv::Point2f> imagePoints1 = projectYaw(Points3d, yaw1, armor_center, sec, nsec);
        std::vector<cv::Point2f> imagePoints2 = projectYaw(Points3d, yaw2, armor_center, sec, nsec);
        double loss1 = - loss_f(imagePoints1, detectedcontour);
        double loss2 = - loss_f(imagePoints2, detectedcontour);

        while ((right - left) > 1)
        {
            iter_count++;
            if (loss1 < loss2)
            {
            right = mid2;
            mid2 = mid1;
            yaw2 = yaw1;
            loss2 = loss1;
            mid1 = left + fibonacciDict[right - left].first;
            yaw1 = start + mid1;
            imagePoints1 = projectYaw(Points3d, yaw1, armor_center, sec, nsec);
            loss1 = - loss_f(imagePoints1, detectedcontour);
            }
            else
            {
            left = mid1;
            mid1 = mid2;
            yaw1 = yaw2;
            loss1 = loss2;
            mid2 = left + fibonacciDict[right - left].second;
            yaw2 = start + mid2;
            imagePoints2 = projectYaw(Points3d, yaw2, armor_center, sec, nsec);
            loss2 = - loss_f(imagePoints2, detectedcontour);
            }
        }

        float optim_yaw = (yaw1 + yaw2) / 2;


        auto end_time = std::chrono::high_resolution_clock::now();
        long long duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        float error = abs(optim_yaw - origin_yaw);
        static std::vector<float> error_vec;  
        static std::vector<long long> time_vec;    
        static std::vector<float> origin_yaw_vec;  
        static std::vector<float> optim_yaw_vec;
        static std::vector<double> loss_vec;
        error_vec.push_back(error);  
        time_vec.push_back(duration);  
        origin_yaw_vec.push_back(origin_yaw);
        loss_vec.push_back(loss1);
        optim_yaw_vec.push_back(optim_yaw);
        
        // std:: cout << "\nbegin=============" << std::endl; 
        // std:: cout << "iter_count                  : " << iter_count << std::endl; 
        // std::cout <<  "abs(optim_yaw - origin_yaw) : " << optim_yaw <<  " - " << origin_yaw << " = "<< abs(optim_yaw - origin_yaw) << std::endl;
        // std::cout <<  "time(us)                    : " << duration << std::endl;    
        // std:: cout << "end=============\n" << std::endl;   

        // if(error_vec.size()%200 == 0){    
        //   std::cout << "==========originyaw  vec:" << std::endl; 
        //   for(auto origin_yaw : origin_yaw_vec){  
        //     std::cout << origin_yaw << " ";
        //   }
        //   std::cout << "==========error  vec:" << std::endl; 
        //   for(auto error : error_vec){
        //     std::cout << error << " ";
        //   }
        //   std::cout << std::endl;
        //   std::cout << "==========time   vec:" << std::endl; 
        //   for(auto time : time_vec){
        //     std::cout << time << " ";
        //   }  
        //   std::cout << std::endl;
        //   std::cout << "==========loss   vec:" << std::endl;
        //   for(auto loss : loss_vec){
        //     std::cout << loss << " ";
        //   }
        //   std::cout << std::endl;
        //   std::cout << "==========optim_yaw   vec:" << std::endl;
        //   for(auto optim_yaw : optim_yaw_vec){
        //     std::cout << optim_yaw << " ";
        //   }
        // }

        return optim_yaw;
    }

    double Optimization::IoU(const std::vector<cv::Point2f>& projectedContour, const std::vector<cv::Point2f>& detectedContour)
    {
        // 假设你已经有了两个凸多边形
        std::vector<cv::Point> poly1, poly2;
        for (auto& point : projectedContour)
        {
            poly1.push_back(cv::Point(point.x, point.y));
        }
        for (auto& point : detectedContour)
        {
            poly2.push_back(cv::Point(point.x, point.y));
        }
        std::vector<cv::Point> all_points(poly1.begin(), poly1.end());
        all_points.insert(all_points.end(), poly2.begin(), poly2.end());

        cv::Rect rect = cv::boundingRect(cv::Mat(all_points)); // 获取最大外接矩形
        // std::cout << "Bounding rect: " << rect << std::endl;

        // 创建三个与外接矩形相同大小的矩阵
        cv::Mat MA(rect.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat MB(rect.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat MC(rect.size(), CV_8UC1, cv::Scalar(0));

        // 将多边形点坐标转换为矩形内相对坐标
        std::vector<cv::Point> poly1_rect, poly2_rect;
        for (const auto& pt : poly1) {
            poly1_rect.push_back({ pt.x - rect.x, pt.y - rect.y });
        }
        for (const auto& pt : poly2) {
            poly2_rect.push_back({ pt.x - rect.x, pt.y - rect.y });
        }

        // 使用多边形填充矩阵
        cv::fillConvexPoly(MA, poly1_rect.data(), static_cast<int>(poly1.size()), cv::Scalar(1), cv::LINE_AA);
        cv::fillConvexPoly(MB, poly2_rect.data(), static_cast<int>(poly2.size()), cv::Scalar(1), cv::LINE_AA);

        // 计算交集矩阵
        cv::bitwise_and(MA, MB, MC);

        // 统计矩阵中1的个数（即面积）
        int area_A = cv::countNonZero(MA);
        int area_B = cv::countNonZero(MB);
        int area_intersection = cv::countNonZero(MC);

        // 计算IOU
        double IOU = static_cast<double>(area_intersection) / (area_A + area_B - area_intersection);

        return IOU;
    }

    double Optimization::loss_f(const std::vector<cv::Point2f>& projectedContour, const std::vector<cv::Point2f>& detectedContour)
    {
        double iou = IoU(projectedContour, detectedContour);
        return iou;
    }

    /**
     * @brief yaw是角度制
     * 
     * @param tvec 
     * @param yaw 
     */
    std::vector<cv::Point2f> Optimization::projectYaw(std::vector<cv::Point3f> objectPoints, float yaw, cv::Point3f armor_center, int32_t sec, int32_t nsec)
    {
        tf2::Quaternion q;
        yaw = deg2rad(yaw);
        std::vector<cv::Point2f> imagePoints;

        q.setRPY(0, - 0.26, yaw);
        tf2::Transform transform(q, tf2::Vector3(armor_center.x, armor_center.y, armor_center.z));
        for (auto& point : objectPoints)
        {
            tf2::Vector3 tf_point;
            tf_point.setX(point.x);
            tf_point.setY(point.y);
            tf_point.setZ(point.z);
            tf_point = transform * tf_point;
            point.x = tf_point.x();
            point.y = tf_point.y();
            point.z = tf_point.z();
        }

        for (auto & point : objectPoints)
        {
            geometry_msgs::msg::PoseStamped temp_pose_point;
            temp_pose_point.header.stamp.sec = sec;
            temp_pose_point.header.stamp.nanosec = nsec;
            temp_pose_point.header.frame_id = target_frame_;
            temp_pose_point.pose.position.x = point.x;
            temp_pose_point.pose.position.y = point.y;
            temp_pose_point.pose.position.z = point.z;
            temp_pose_point.pose.orientation.x = 0;
            temp_pose_point.pose.orientation.y = 0;
            temp_pose_point.pose.orientation.z = 0;
            temp_pose_point.pose.orientation.w = 1;
            try {
                temp_pose_point = tf2_buffer_->transform(temp_pose_point, "camera_link");
                temp_pose_point.header.frame_id = target_frame_;
                point.x = temp_pose_point.pose.position.x;
                point.y = temp_pose_point.pose.position.y;
                point.z = temp_pose_point.pose.position.z;
            } catch (const tf2::ExtrapolationException & ex) {
                std::cout << "ExtrapolationException" << std::endl;
            }
        }

        cv::Mat rvec, tvec;
        tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
        rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);

        cv::projectPoints(objectPoints, rvec, tvec, this->camera_matrix_, this->dist_coeffs_, imagePoints);
        // for (auto & point : objectPoints)
        // {
        //   cv::Mat point_3d(4, 1, CV_64F);
        //   point_3d.at<double>(0) = point.x;
        //   point_3d.at<double>(1) = point.y;
        //   point_3d.at<double>(2) = point.z;
        //   point_3d.at<double>(3) = 1;
        //   cv::Mat point_2d = this->project_matrix_ * cv::Mat(point_3d);
        //   imagePoints.push_back(cv::Point2f(point_2d.at<double>(0) / point_2d.at<double>(2), point_2d.at<double>(1) / point_2d.at<double>(2)));
        // }

        return imagePoints;
    }
}