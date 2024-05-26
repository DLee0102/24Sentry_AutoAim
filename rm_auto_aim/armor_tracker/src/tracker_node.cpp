// Copyright 2022 Chen Jun
#include "armor_tracker/tracker_node.hpp"

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  declareParameters();

  initDebugTools();

  omni_strategy_ = std::make_unique<OmniPercepStrategy>(hit_order_);
  omni_strategy_->setTrackedArmor("negative");

  tracker_ = std::make_unique<Tracker>(
    outpost_max_match_distance_, outpost_max_match_yaw_diff_, hit_order_, lock_thres_,
    outpost_sqxyz_, outpost_sqyaw_, outpost_sqr_, outpost_r_xyz_factor_, outpost_r_yaw_,
    outpost_yaw_angle_thres_, normal_max_match_distance_, normal_max_match_yaw_diff_, normal_sqxyz_,
    normal_sqyaw_, normal_sqr_, normal_r_xyz_factor_, normal_r_yaw_, normal_yaw_angle_thres_);

  tracker_->tracking_thres = tracking_thres_;

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // 1.13 changed from std::chrono::duration<int>(1) to std::chrono::milliseconds(10)
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

  tracker_->optimize_yaw_->setTargetFrame(target_frame_);
  tracker_->optimize_yaw_->setTfBuffer(tf2_buffer_);
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      tracker_->optimize_yaw_->setCamerainfo(camera_info->k, camera_info->d, camera_info->p);
      cam_info_sub_.reset();
    });

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS());

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);

  trajectory_slover_ =
    std::make_shared<TrajectorySlover>(max_iter_, stop_error_, R_K_iter_, init_speed_, is_hero_);

  bullet_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/bullet_speed", 10, std::bind(&ArmorTrackerNode::setBulletSpeed, this, std::placeholders::_1));

  latency_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/latency", 10, std::bind(&ArmorTrackerNode::setLatancy, this, std::placeholders::_1));

  auto_aim_restartflag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/auto_aim_restartflag", 10,
    std::bind(&ArmorTrackerNode::setAutoAimRestartflag, this, std::placeholders::_1));

  // control gimbal
  nav_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
    "/detector/navarmors", rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::navCallback, this, std::placeholders::_1));
  omni_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
    "/detector/omniarmors", rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::omniCallback, this, std::placeholders::_1));
  gimbal_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&ArmorTrackerNode::gimbalStateCallback, this, std::placeholders::_1));

  // params_callback_handle_ = this->add_on_set_parameters_callback(
  //   std::bind(&ArmorTrackerNode::parametersCallback, this, std::placeholders::_1));
}

void ArmorTrackerNode::declareParameters()
{
  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // Tracker
  outpost_max_match_distance_ = this->declare_parameter("outpost.max_match_distance", 0.2);
  outpost_max_match_yaw_diff_ = this->declare_parameter("outpost.max_match_yaw_diff", 0.5);
  normal_max_match_distance_ = this->declare_parameter("normal.max_match_distance", 0.2);
  normal_max_match_yaw_diff_ = this->declare_parameter("normal.max_match_yaw_diff", 0.5);

  lock_thres_ = this->declare_parameter("tracker.lock_thres", 3.0);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);
  tracking_thres_ = this->declare_parameter("tracker.tracking_thres", 5);

  hit_order_ = {{"1", 0},       {"2", 2},      {"3", 3},     {"4", 6},        {"5", 4},
                {"outpost", 7}, {"guard", 11}, {"base", 12}, {"negative", 13}};
  hit_order_["1"] = this->declare_parameter("hit_order.1", 0);
  hit_order_["2"] = this->declare_parameter("hit_order.2", 2);
  hit_order_["3"] = this->declare_parameter("hit_order.3", 3);
  hit_order_["4"] = this->declare_parameter("hit_order.4", 6);
  hit_order_["5"] = this->declare_parameter("hit_order.5", 4);
  hit_order_["outpost"] = this->declare_parameter("hit_order.outpost", 7);
  hit_order_["guard"] = this->declare_parameter("hit_order.guard", 11);
  hit_order_["base"] = this->declare_parameter("hit_order.base", 12);
  hit_order_["negative"] = this->declare_parameter("hit_order.negative", 13);

  // omni perception
  pitch_limit_ = this->declare_parameter<std::vector<double>>("omniperception.pitch_limit", { 25.0, -25.0 });
  inside_threshold_yaw_ = this->declare_parameter("omniperception.inside_threshold_yaw", 20.0);
  inside_threshold_pitch_ = this->declare_parameter("omniperception.inside_threshold_pitch", 5.0);
  spin_target_angle_ =
    this->declare_parameter("omniperception.spin_target_angle", 120.0);

  // Debug
  debug_ = this->declare_parameter("debug", true);

  // update_Q - process noise covariance matrix
  outpost_sqxyz_ = this->declare_parameter("outpost.ekf.sigma2_q_xyz", 0.01);
  outpost_sqyaw_ = this->declare_parameter("outpost.ekf.sigma2_q_yaw", 0.01);
  outpost_sqr_ = this->declare_parameter("outpost.ekf.sigma2_q_r", 0.01);
  normal_sqxyz_ = this->declare_parameter("normal.ekf.sigma2_q_xyz", 0.01);
  normal_sqyaw_ = this->declare_parameter("normal.ekf.sigma2_q_yaw", 0.01);
  normal_sqr_ = this->declare_parameter("normal.ekf.sigma2_q_r", 0.01);

  // update_R - measurement noise covariance matrix
  outpost_r_xyz_factor_ = declare_parameter("outpost.ekf.r_xyz_factor", 0.02);
  outpost_r_yaw_ = declare_parameter("outpost.ekf.r_yaw", 0.02);
  normal_r_xyz_factor_ = declare_parameter("normal.ekf.r_xyz_factor", 0.02);
  normal_r_yaw_ = declare_parameter("normal.ekf.r_yaw", 0.02);

  //trajectory slover param
  max_iter_ = this->declare_parameter("trajectory.max_iter", 10);
  stop_error_ = this->declare_parameter("trajectory.stop_error", 0.001);
  R_K_iter_ = this->declare_parameter("trajectory.R_K_iter", 50);
  init_speed_ = this->declare_parameter("trajectory.init_bullet_speed", 26.5);
  is_hero_ = this->declare_parameter("trajectory.is_hero", false);
  static_offset_yaw_ = this->declare_parameter("trajectory.static_offset.yaw", 0.0);
  static_offset_pitch_ = this->declare_parameter("trajectory.static_offset.pitch", 0.0);

  //Get fire angle thres
  outpost_yaw_angle_thres_ = this->declare_parameter("outpost.yaw_angle_thres", 0.5);
  normal_yaw_angle_thres_ = this->declare_parameter("normal.yaw_angle_thres", 0.5);

  fire_permit_thres_ = this->declare_parameter("fire_permit_thres", 1.5);
  fire_latency_ = this->declare_parameter("fire_latency", 0.02);
  min_speed_ = this->declare_parameter("min_speed", 20.0);
}

void ArmorTrackerNode::initDebugTools()
{
  if (debug_) {
    createDebugPublishers();  // Debug param change moniter
    debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    debug_cb_handle_ =
      debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
        debug_ = p.as_bool();
        debug_ ? createDebugPublishers() : destroyDebugPublishers();
      });
  }

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  pred_armor_marker_.ns = "pred_armors";
  pred_armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  pred_armor_marker_.scale.x = 0.03;
  pred_armor_marker_.scale.z = 0.125;
  pred_armor_marker_.color.a = 1.0;
  pred_armor_marker_.color.g = 1.0;
  trajectory_marker_.ns = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  trajectory_marker_.scale.x = 0.01;  // Set the scale of the spheres
  trajectory_marker_.scale.y = 0.01;
  trajectory_marker_.scale.z = 0.01;
  trajectory_marker_.color.a = 1.0;  // Set alpha (transparency)
  trajectory_marker_.color.r = 1.0;  // Set color to red
  trajectory_marker_.color.g = 0.0;
  trajectory_marker_.color.b = 0.0;
}

// rcl_interfaces::msg::SetParametersResult ArmorTrackerNode::parametersCallback(
//   const std::vector<rclcpp::Parameter> & parameters)
// {
//     rcl_interfaces::msg::SetParametersResult result;
//     result.successful = true;
//     for (const auto & param : parameters) {
//       if (param.get_name() == "ekf.sigma2_q_xyz" || param.get_name() == "ekf.sigma2_q_yaw" ||
//           param.get_name() == "ekf.sigma2_q_r")
//       {
//         if (param.get_name() == "ekf.sigma2_q_xyz") {
//           this->s2qxyz_ = param.as_double();
//         } else if (param.get_name() == "ekf.sigma2_q_yaw") {
//           this->s2qyaw_ = param.as_double();
//         } else if (param.get_name() == "ekf.sigma2_q_r") {
//           this->s2qr_ = param.as_double();
//         }
//         auto u_q = [this]() {
//           Eigen::MatrixXd q(6, 6);
//           double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
//           double q_x_x = pow(t, 4) / 4 * x;
//           double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
//           double q_r = pow(t, 4) / 4 * r;
//           // clang-format off
//           //    xc      yc     za      yaw     v_yaw   r
//           q <<  q_x_x,  0,     0,       0,      0,      0,
//                 0,      q_x_x, 0,       0,      0,      0,
//                 0,      0,     q_x_x,   0,      0,      0,
//                 0,      0,     0,     q_y_y,  q_y_vy,   0,
//                 0,      0,     0,     q_y_vy, q_vy_vy,  0,
//                 0,      0,     0,     0,      0,       q_r;
//           // clang-format on
//           return q;
//         };
//         this->tracker_->observer_->ekf_.setUpdateQ(u_q);
//       }
//       else if (param.get_name() == "ekf.r_xyz_factor" || param.get_name() == "ekf.r_yaw")
//       {
//         if (param.get_name() == "ekf.r_xyz_factor") {
//           this->r_xyz_factor_ = param.as_double();
//         } else if (param.get_name() == "ekf.r_yaw") {
//           this->r_yaw_ = param.as_double();
//         }
//         auto u_r = [this](const Eigen::VectorXd & z) {
//           Eigen::DiagonalMatrix<double, 4> r;
//           double x = r_xyz_factor_;
//           r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw_;
//           return r;
//         };
//         this->tracker_->observer_->ekf_.setUpdateR(u_r);
//       }
//     }
//     return result;
// }

void ArmorTrackerNode::createDebugPublishers()
{
  // Debug information
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
}

void ArmorTrackerNode::destroyDebugPublishers() { marker_pub_.reset(); }

/* 若接收到指定状态量，则重置跟踪器状态 */
void ArmorTrackerNode::setAutoAimRestartflag(
  const std_msgs::msg::Bool::SharedPtr auto_aim_restartflag)
{
  if (auto_aim_restartflag->data == true) {
    tracker_->resetState();
  }
}

void ArmorTrackerNode::setBulletSpeed(const std_msgs::msg::Float64::SharedPtr bulletspeed)
{
  if (bulletspeed->data != 0) {
    auto diff = bulletspeed->data - trajectory_slover_->getBulletSpeed();
    if ((diff > 0.2 || diff < -0.2) && bulletspeed->data > min_speed_) {
      trajectory_slover_->setBulletSpeed(bulletspeed->data);
      RCLCPP_INFO(
        this->get_logger(), "set bullet speed: %.3f", trajectory_slover_->getBulletSpeed());
    }
  }
}

void ArmorTrackerNode::setLatancy(const std_msgs::msg::Float64::SharedPtr latency)
{
  if (latency->data >= 0) {
    latency_ = latency->data;
  }
}

void ArmorTrackerNode::gimbalStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
  current_state_.pitch = -rad2deg(joint_state->position[0]);
  current_state_.yaw = -rad2deg(joint_state->position[1]);
}

void ArmorTrackerNode::navCallback(const auto_aim_interfaces::msg::Armors::SharedPtr nav_msg)
{
  if (nav_msg->header.frame_id == "gimbal_odom" && !nav_control_flag_) {
    omni_strategy_->init(nav_msg);
    nav_armor_ = omni_strategy_->getresultArmor();
    if (omni_strategy_->switchTO(nav_armor_.number)) {
      omni_strategy_->setTrackedArmor(nav_armor_.number);
      std::cout << "nav armor:" << nav_armor_.number << std::endl;
      Eigen::Vector2d angel_diff_temp = calcYawAndPitch(Eigen::Vector3d(
        nav_msg->armors[0].pose.position.x, nav_msg->armors[0].pose.position.y,
        nav_msg->armors[0].pose.position.z));
      target_state_.yaw = rad2deg((double)angel_diff_temp(0));
      while (target_state_.yaw < -180.0) target_state_.yaw += 360.0;
      while (target_state_.yaw > 180.0) target_state_.yaw -= 360.0;
      target_state_.pitch = -rad2deg((double)angel_diff_temp(1));
      if (target_state_.pitch > pitch_limit_[0]) target_state_.pitch = pitch_limit_[0];
      if (target_state_.pitch < pitch_limit_[1]) target_state_.pitch = pitch_limit_[1];
      bool inside_offset =
        abs(target_state_.yaw - current_state_.yaw) < inside_threshold_yaw_ &&
        abs(target_state_.pitch - current_state_.pitch) < inside_threshold_pitch_;
      if (!inside_offset) {
        nav_control_flag_ = true;
      }
    }
  }
}

void ArmorTrackerNode::omniCallback(const auto_aim_interfaces::msg::Armors::SharedPtr omni_msg)
{
  if (omni_msg->header.frame_id == "camera_link" && !omni_control_flag_) {
    omni_strategy_->init(omni_msg);
    fulldetect_armor_ = omni_strategy_->getresultArmor();
    std::cout << "full detect armor: " << fulldetect_armor_.number << "    tracked id: "
              << omni_strategy_->tracked_id_ << std::endl;
    if (omni_strategy_->switchTO(fulldetect_armor_.number)) {
      std::cout << "full detect set tracked id to " << fulldetect_armor_.number << std::endl;
      omni_strategy_->setTrackedArmor(fulldetect_armor_.number);
      if (fulldetect_armor_.header.frame_id == "usb_camera_a_link") {
        target_state_.yaw = current_state_.yaw + spin_target_angle_;
        while (target_state_.yaw < -180.0) target_state_.yaw += 360.0;
        while (target_state_.yaw > 180.0) target_state_.yaw -= 360.0;
        target_state_.pitch = 0.0;
      } else if (fulldetect_armor_.header.frame_id == "usb_camera_b_link") {
        target_state_.yaw = current_state_.yaw - spin_target_angle_;
        while (target_state_.yaw < -180.0) target_state_.yaw += 360.0;
        while (target_state_.yaw > 180.0) target_state_.yaw -= 360.0;
        target_state_.pitch = 0.0; //TODO: set pitch according to full detect's msg
      }
      bool inside_offset =
        abs(target_state_.yaw - current_state_.yaw) < inside_threshold_yaw_ &&
        abs(target_state_.pitch - current_state_.pitch) < inside_threshold_pitch_;
      if (!inside_offset) {
        omni_control_flag_ = true;
      }
    }
  }
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // RCLCPP_WARN(this->get_logger(), "------------------Latency after armorsCallback: %f", (this->now() - armors_msg->header.stamp).seconds() * 1000);
  // if (armors_msg->fulldetect)
  // {
  //   auto_aim_interfaces::msg::Target fulldetec_target_msg;
  //   auto_aim_interfaces::msg::Armor fulldetect_armor_;

  //   omni_strategy_->init(armors_msg);
  //   fulldetect_armor_ = omni_strategy_->getresultArmor();
  //   if (!omni_strategy_->switchTO(fulldetect_mor.number))
  //   {
  //     return;
  //   }

  //   if (armors_msg->armors[0].header.frame_id == "usb_camera_a_link")
  //   {
  //     fulldetec_target_msg.header = armors_msg->header;
  //     fulldetec_target_msg.control = false;
  //     fulldetec_target_msg.fire_permit = 0;
  //     fulldetec_target_msg.offset_yaw = spin_target_angle_;
  //     fulldetec_target_msg.offset_pitch = 0.0f;
  //     target_pub_->publish(fulldetec_target_msg);
  //   }
  //   else if (armors_msg->armors[0].header.frame_id == "usb_camera_b_link")
  //   {
  //     fulldetec_target_msg.header = armors_msg->header;
  //     fulldetec_target_msg.control = false;
  //     fulldetec_target_msg.fire_permit = 0;
  //     fulldetec_target_msg.offset_yaw = -spin_target_angle_;
  //     fulldetec_target_msg.offset_pitch = 0.0f;
  //     target_pub_->publish(fulldetec_target_msg);
  //   }
  //   else if (armors_msg->armors[0].header.frame_id == "base_link")
  //   {
  //     for (auto & armor : armors_msg->armors) {
  //       geometry_msgs::msg::PoseStamped ps;
  //       ps.header = armors_msg->header;
  //       ps.pose = armor.pose;
  //       try {
  //         armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
  //       } catch (const tf2::ExtrapolationException & ex) {
  //         RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
  //         return;
  //       }
  //     }

  //     Eigen::Vector2d angel_diff_temp = calcYawAndPitch(Eigen::Vector3d(armors_msg->armors[0].pose.position.x, armors_msg->armors[0].pose.position.y, armors_msg->armors[0].pose.position.z));
  //     fulldetec_target_msg.header = armors_msg->header;
  //     fulldetec_target_msg.control = false;
  //     fulldetec_target_msg.fire_permit = 0;
  //     fulldetec_target_msg.offset_yaw = rad2deg((double)angel_diff_temp(0));
  //     fulldetec_target_msg.offset_pitch = rad2deg((double)angel_diff_temp(1));
  //     target_pub_->publish(fulldetec_target_msg);
  //   }

  //   usleep(500);
  //   return;
  // }

  // control gimbal
  if (nav_control_flag_) {
    auto_aim_interfaces::msg::Target gimbal_control_target;
    gimbal_control_target.header = armors_msg->header;
    gimbal_control_target.control = true;
    gimbal_control_target.tracking = false;   
    gimbal_control_target.fire_permit = 0;
    gimbal_control_target.offset_yaw = target_state_.yaw - current_state_.yaw;
    while (gimbal_control_target.offset_yaw > 180.0) gimbal_control_target.offset_yaw -= 360.0;
    while (gimbal_control_target.offset_yaw < -180.0) gimbal_control_target.offset_yaw += 360.0;
    gimbal_control_target.offset_pitch =
      -target_state_.pitch + current_state_.pitch;  // TODO: wrong direction?
    std::cout << "[ nav control ]" << std::endl;
    std::cout << "Target yaw: " << target_state_.yaw << "Current yaw: " << current_state_.yaw
              << std::endl;
    std::cout << "Target pitch: " << target_state_.pitch
              << "Current pitch: " << current_state_.pitch << std::endl;
    if (abs(gimbal_control_target.offset_yaw) < 5 && abs(gimbal_control_target.offset_pitch) < 5) {
      nav_control_flag_ = false;
    }
    tracker_->resetState();
    target_pub_->publish(gimbal_control_target);
    return;
  } else if (omni_control_flag_) {
    auto_aim_interfaces::msg::Target gimbal_control_target;
    gimbal_control_target.header = armors_msg->header;
    gimbal_control_target.control = true;
    gimbal_control_target.tracking = false;   
    gimbal_control_target.fire_permit = 0;
    gimbal_control_target.offset_yaw = target_state_.yaw - current_state_.yaw;
    while (gimbal_control_target.offset_yaw > 180.0) gimbal_control_target.offset_yaw -= 360.0;
    while (gimbal_control_target.offset_yaw < -180.0) gimbal_control_target.offset_yaw += 360.0;
    gimbal_control_target.offset_pitch =
      -target_state_.pitch + current_state_.pitch;  // TODO: wrong direction?
    std::cout << "[ full detect control ]" << std::endl;
    std::cout << "Target yaw: " << target_state_.yaw << "Current yaw: " << current_state_.yaw
              << std::endl;
    std::cout << "Target pitch: " << target_state_.pitch
              << "Current pitch: " << current_state_.pitch << std::endl;
    if (abs(gimbal_control_target.offset_yaw) < 5 && abs(gimbal_control_target.offset_pitch) < 5) {
      omni_control_flag_ = false;
    }
    tracker_->resetState();
    target_pub_->publish(gimbal_control_target);
    return;
  }

  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
               Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                 max_armor_distance_;
      }),
    armors_msg->armors.end());

  auto_aim_interfaces::msg::Target target_msg_final;
  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  target_msg_final.header.stamp = time;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST || tracker_->tracker_state == Tracker::SWITCH) {
    tracker_->init(armors_msg);
    // std::cout<< "tracker id" << tracker_->tracked_id << std::endl;
    // if (tracker_->tracked_id != "") {
    //   omni_strategy_->setTrackedArmor(tracker_->tracked_id);
    // } else {
    omni_strategy_->setTrackedArmor("negative");
    // }

    target_msg.tracking = false;
  } else {
    omni_strategy_->setTrackedArmor(tracker_->tracked_id);
    dt_ = (time - last_time_).seconds();
    // RCLCPP_WARN(this->get_logger(), "dt: %f", dt_ * 1000);
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);

    tracker_->update(armors_msg, dt_);

    // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = tracker_->info_yaw_diff;
    info_msg.position.x = tracker_->observer_->measurement_(0);
    info_msg.position.y = tracker_->observer_->measurement_(1);
    info_msg.position.z = tracker_->observer_->measurement_(2);
    info_msg.yaw = tracker_->observer_->measurement_(3);
    info_pub_->publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      // Fill target message
      const auto & state = tracker_->observer_->target_state_;
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->observer_->tracked_armors_num_);
      if (tracker_->observer_->tracked_armors_num_ == ArmorsNum::OUTPOST_3) {
        target_msg.position.x = state(0);
        target_msg.position.y = state(1);
        target_msg.position.z = state(2);
        target_msg.yaw = state(3);
        target_msg.v_yaw = state(4);
        target_msg.radius_1 = state(5);
      } else {
        target_msg.position.x = state(0);
        target_msg.velocity.x = state(1);
        target_msg.position.y = state(2);
        target_msg.velocity.y = state(3);
        target_msg.position.z = state(4);
        target_msg.velocity.z = state(5);
        target_msg.yaw = state(6);
        target_msg.v_yaw = state(7);
        target_msg.radius_1 = state(8);
        target_msg.radius_2 = tracker_->observer_->another_r_;
        target_msg.dz = tracker_->observer_->dz_;
      }
    }
  }

  last_time_ = time;

  if (target_msg.tracking == true) {
    //save target states
    Eigen::Vector3d now_car_pos =
      Eigen::Vector3d(target_msg.position.x, target_msg.position.y, target_msg.position.z);

    //save the pred target
    double pred_dt =
      fire_latency_ + latency_ / 1000 +
      (now_car_pos.norm() - target_msg.radius_1) / trajectory_slover_->getBulletSpeed();

    Eigen::Vector3d armor_target;
    double min_yaw_diff = DBL_MAX;
    bool tracked_permit = false;
    tracker_->observer_->getTargetArmorPosition(
      pred_dt, armor_target, min_yaw_diff, tracked_permit);

    geometry_msgs::msg::PoseStamped temp_pose_point;
    Eigen::Vector3d armor_target_pitch_link;
    temp_pose_point.header.stamp = target_msg.header.stamp;
    temp_pose_point.header.frame_id = target_msg.header.frame_id;
    temp_pose_point.pose.position.x = armor_target(0);
    temp_pose_point.pose.position.y = armor_target(1);
    temp_pose_point.pose.position.z = armor_target(2);
    temp_pose_point.pose.orientation.x = 0;
    temp_pose_point.pose.orientation.y = 0;
    temp_pose_point.pose.orientation.z = 0;
    temp_pose_point.pose.orientation.w = 1;
    try {
      temp_pose_point = tf2_buffer_->transform(temp_pose_point, "shoot_link");
      temp_pose_point.header.frame_id = target_frame_;
      armor_target_pitch_link(0) = temp_pose_point.pose.position.x;
      armor_target_pitch_link(1) = temp_pose_point.pose.position.y;
      armor_target_pitch_link(2) = temp_pose_point.pose.position.z;
    } catch (const tf2::ExtrapolationException & ex) {
      std::cout << "ExtrapolationException" << std::endl;
    }

    //Get offset
    Eigen::Vector2d angel_diff = calcYawAndPitch(armor_target_pitch_link);

    auto trajectory_pitch = (-trajectory_slover_->calcPitchCompensate(armor_target));

    auto trajectory_view = trajectory_slover_->getTrajectoryWorld();

    //Set fire permit
    int8_t fire_permit = 0;
    if (fabs(angel_diff[0]) < fire_permit_thres_ && tracked_permit) fire_permit = 1;

    target_msg.fire_permit = fire_permit;
    target_msg.offset_yaw = rad2deg((double)angel_diff(0)) + static_offset_yaw_;
    target_msg.offset_pitch =
      rad2deg((double)angel_diff(1)) + trajectory_pitch + static_offset_pitch_;

    RCLCPP_INFO(this->get_logger(), "trajectory_pitch : %lf", trajectory_pitch);
    RCLCPP_INFO(
      this->get_logger(), "offset_pitch : %lf , offset_yaw : %lf", target_msg.offset_pitch,
      target_msg.offset_yaw);
    RCLCPP_INFO(this->get_logger(), "fire_permit : %d", fire_permit);
    RCLCPP_INFO(get_logger(), "distance : %lf", (now_car_pos.norm() - target_msg.radius_1));
    RCLCPP_INFO(get_logger(), "speed : %lf", trajectory_slover_->getBulletSpeed());
    RCLCPP_INFO(get_logger(), "latency : %lf", latency_ / 1000.0);
    if ((!(isnan(target_msg.offset_yaw) || isnan(target_msg.offset_pitch))) && tracked_permit) {
      target_msg_final = target_msg;
      target_msg_final.control = true;
    }

    if (debug_) publishMarkers(target_msg, trajectory_view);
  }

  target_pub_->publish(target_msg_final);
}

void ArmorTrackerNode::publishMarkers(
  const auto_aim_interfaces::msg::Target & target_msg,
  const std::vector<Eigen::Vector3d> & trajectory_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;
  pred_armor_marker_.header = target_msg.header;
  trajectory_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.points.clear();
    armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;

    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armor_marker_.id = i;
      armor_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);

      marker_array.markers.emplace_back(armor_marker_);
    }
    //pred_armor visualization

    pred_armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    pred_armor_marker_.points.clear();
    pred_armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;

    tf2::Quaternion q;
    q.setRPY(
      0, target_msg.id == "outpost" ? -0.26 : 0.26, tracker_->observer_->target_armor_pos_.second);
    pred_armor_marker_.pose.position.x = tracker_->observer_->target_armor_pos_.first.x();
    pred_armor_marker_.pose.position.y = tracker_->observer_->target_armor_pos_.first.y();
    pred_armor_marker_.pose.position.z = tracker_->observer_->target_armor_pos_.first.z();
    pred_armor_marker_.pose.orientation = tf2::toMsg(q);
    marker_array.markers.emplace_back(pred_armor_marker_);

    //trajectory visualization
    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();
    trajectory_marker_.points.reserve(trajectory_msg.size());
    for (const auto & point : trajectory_msg) {
      geometry_msgs::msg::Point p;
      p.x = point[0];
      p.y = point[1];
      p.z = point[2];
      trajectory_marker_.points.emplace_back(p);
    }
    marker_array.markers.emplace_back(trajectory_marker_);
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    trajectory_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    pred_armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.emplace_back(armor_marker_);
    marker_array.markers.emplace_back(pred_armor_marker_);
    marker_array.markers.emplace_back(trajectory_marker_);
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)
