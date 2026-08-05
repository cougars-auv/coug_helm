// Copyright (c) 2026 BYU FROST Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file bt_helm.cpp
 * @brief Implementation of the BTHelmNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_helm/bt_helm.hpp"

#include <GeographicLib/LocalCartesian.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

#include "coug_helm/bt_nodes/back_up.hpp"
#include "coug_helm/bt_nodes/compute_home_waypoint.hpp"
#include "coug_helm/bt_nodes/compute_surface_waypoint.hpp"
#include "coug_helm/bt_nodes/disarm_thruster.hpp"
#include "coug_helm/bt_nodes/emergency_surface.hpp"
#include "coug_helm/bt_nodes/follow_waypoints.hpp"
#include "coug_helm/bt_nodes/is_odom_healthy.hpp"
#include "coug_helm/bt_nodes/is_origin_set.hpp"
#include "coug_helm/bt_nodes/is_waypoints_received.hpp"
#include "coug_helm/bt_nodes/load_behavior.hpp"
#include "coug_helm/bt_nodes/load_waypoints.hpp"
#include "coug_helm/bt_nodes/progress_checker.hpp"
#include "coug_helm/bt_nodes/recovery_node.hpp"
#include "coug_helm/bt_nodes/reset_localization.hpp"
#include "coug_helm/bt_nodes/round_robin.hpp"
#include "coug_helm/bt_nodes/stop.hpp"
#include "coug_helm/bt_nodes/wait.hpp"
#include "coug_helm/utils/behavior_enums.hpp"

namespace coug_helm {

using utils::Behavior;
using utils::toString;
using utils::Waypoint;

BTHelmNode::BTHelmNode(const rclcpp::NodeOptions& options)
    : Node("bt_helm_node", options),
      diagnostic_updater_(this),
      local_cartesian_(0.0, 0.0, 0.0, GeographicLib::Geocentric::WGS84()) {
  param_listener_ = std::make_shared<bt_helm_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- Blackboard ---
  blackboard_ = BT::Blackboard::create();

  blackboard_->set("node", static_cast<rclcpp::Node*>(this));
  blackboard_->set("hsd_topic", params_.hsd_topic);
  blackboard_->set("arm_thruster_service", params_.arm_thruster_service);
  blackboard_->set("reset_localization_service", params_.reset_localization_service);

  blackboard_->set("pending_behavior", static_cast<int>(Behavior::STOP));
  blackboard_->set("active_behavior", static_cast<int>(Behavior::STOP));

  blackboard_->set("current_waypoint", size_t{0});
  blackboard_->set("prev_norm_dist", -1.0);

  blackboard_->set("active_waypoints", std::vector<Waypoint>{});
  blackboard_->set("mission_waypoints", std::vector<Waypoint>{});

  blackboard_->set("current_x", 0.0);
  blackboard_->set("current_y", 0.0);
  blackboard_->set("current_z", 0.0);
  blackboard_->set("origin_set", false);
  blackboard_->set("last_odom_time", 0.0);
  blackboard_->set("current_time", 0.0);

  blackboard_->set("surface_capture_radius", params_.surface_capture_radius);
  blackboard_->set("surface_capture_radius_z", params_.surface_capture_radius_z);
  blackboard_->set("home_capture_radius", params_.home_capture_radius);
  blackboard_->set("home_capture_radius_z", params_.home_capture_radius_z);
  blackboard_->set("surface_slip_radius", params_.surface_slip_radius);
  blackboard_->set("surface_slip_radius_z", params_.surface_slip_radius_z);
  blackboard_->set("home_slip_radius", params_.home_slip_radius);
  blackboard_->set("home_slip_radius_z", params_.home_slip_radius_z);

  blackboard_->set("default_speed", params_.default_speed_rpm);

  blackboard_->set("odom_timeout_sec", params_.odom_timeout_sec);
  blackboard_->set("odom_recovery_timeout_sec", params_.odom_recovery_timeout_sec);

  blackboard_->set("progress_timeout_sec", params_.progress_timeout_sec);
  blackboard_->set("progress_threshold", params_.progress_threshold);
  blackboard_->set("number_of_retries", static_cast<int>(params_.number_of_retries));
  blackboard_->set("wait_duration_sec", params_.wait_duration_sec);

  // --- ROS Interfaces ---
  origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      params_.origin_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BTHelmNode::originCallback, this, std::placeholders::_1));
  waypoint_sub_ = create_subscription<coug_interfaces::msg::WayPointList>(
      params_.waypoint_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BTHelmNode::waypointCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BTHelmNode::odomCallback, this, std::placeholders::_1));

  start_srv_ = createBehaviorService(params_.start_service, Behavior::MISSION, "Mission");
  stop_srv_ = createBehaviorService(params_.stop_service, Behavior::STOP, "Stop");
  surface_srv_ = createBehaviorService(params_.surface_service, Behavior::SURFACE, "Surface");
  home_srv_ = createBehaviorService(params_.home_service, Behavior::HOME, "Home");
  emergency_stop_srv_ = createBehaviorService(params_.emergency_stop_service,
                                              Behavior::EMERGENCY_STOP, "Emergency stop");
  emergency_surface_srv_ = createBehaviorService(params_.emergency_surface_service,
                                                 Behavior::EMERGENCY_SURFACE, "Emergency surface");

  tick_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / params_.tick_rate_hz),
                                  std::bind(&BTHelmNode::tickTree, this));

  // --- BT Node Registration ---
  factory_.registerNodeType<bt_nodes::IsOdomHealthy>("IsOdomHealthy");
  factory_.registerNodeType<bt_nodes::IsOriginSet>("IsOriginSet");
  factory_.registerNodeType<bt_nodes::IsWaypointsReceived>("IsWaypointsReceived");
  factory_.registerNodeType<bt_nodes::BackUp>("BackUp");
  factory_.registerNodeType<bt_nodes::ComputeHomeWaypoint>("ComputeHomeWaypoint");
  factory_.registerNodeType<bt_nodes::ComputeSurfaceWaypoint>("ComputeSurfaceWaypoint");
  factory_.registerNodeType<bt_nodes::DisarmThruster>("DisarmThruster");
  factory_.registerNodeType<bt_nodes::EmergencySurface>("EmergencySurface");
  factory_.registerNodeType<bt_nodes::FollowWaypoints>("FollowWaypoints");
  factory_.registerNodeType<bt_nodes::LoadBehavior>("LoadBehavior");
  factory_.registerNodeType<bt_nodes::LoadWaypoints>("LoadWaypoints");
  factory_.registerNodeType<bt_nodes::ResetLocalization>("ResetLocalization");
  factory_.registerNodeType<bt_nodes::Stop>("Stop");
  factory_.registerNodeType<bt_nodes::Wait>("Wait");
  factory_.registerNodeType<bt_nodes::RecoveryNode>("RecoveryNode");
  factory_.registerNodeType<bt_nodes::RoundRobin>("RoundRobin");
  factory_.registerNodeType<bt_nodes::ProgressChecker>("ProgressChecker");

  factory_.registerScriptingEnums<Behavior>();

  std::string pkg_share = ament_index_cpp::get_package_share_directory("coug_helm");
  tree_ = factory_.createTreeFromFile(pkg_share + "/trees/bt_helm_tree.xml", blackboard_);

  // --- Groot2 Publisher ---
  if (params_.publish_groot2) {
    groot2_pub_ = std::make_unique<BT::Groot2Publisher>(tree_, params_.groot2_port);
    RCLCPP_INFO(get_logger(), "Groot2 Publisher: Port %ld", params_.groot2_port);
  }

  // --- ROS Diagnostics ---
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/bt_helm_node");

    std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";
    diagnostic_updater_.add(prefix + "Behavior Status", this, &BTHelmNode::checkBehaviorStatus);
  }

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr BTHelmNode::createBehaviorService(
    const std::string& service, Behavior behavior, const std::string& label) {
  return create_service<std_srvs::srv::Trigger>(
      service, [this, behavior, label](const std_srvs::srv::Trigger::Request::SharedPtr,
                                       std_srvs::srv::Trigger::Response::SharedPtr res) {
        tree_.haltTree();
        blackboard_->set("pending_behavior", static_cast<int>(behavior));
        res->success = true;
        res->message = label + " requested.";
      });
}

void BTHelmNode::originCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  if (!origin_set_ && msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    local_cartesian_.Reset(msg->latitude, msg->longitude, msg->altitude);
    origin_set_ = true;
    blackboard_->set("origin_set", true);
    RCLCPP_INFO(get_logger(), "GPS origin set: Lat %.6f, Lon %.6f, Alt %.2f", msg->latitude,
                msg->longitude, msg->altitude);
  }
}

void BTHelmNode::waypointCallback(const coug_interfaces::msg::WayPointList::SharedPtr msg) {
  if (msg->waypoints.empty()) {
    return;
  }

  if (!origin_set_) {
    RCLCPP_WARN(get_logger(), "Origin not set. Dropping mission.");
    return;
  }

  // Transform waypoints from lat/lon into the shared map frame
  std::vector<Waypoint> enu_waypoints;
  for (size_t i = 0; i < msg->waypoints.size(); ++i) {
    const auto& src = msg->waypoints[i];
    const auto& gps = src.position;
    Waypoint wp;
    double dummy_z;
    local_cartesian_.Forward(gps.latitude, gps.longitude, 0.0, wp.position.x, wp.position.y,
                             dummy_z);
    wp.position.z = gps.altitude;
    wp.mode = src.mode;
    wp.speed = src.speed_rpm;
    wp.capture_radius =
        src.capture_radius > 0.0 ? src.capture_radius : params_.default_capture_radius;
    wp.capture_radius_z =
        src.capture_radius_z > 0.0 ? src.capture_radius_z : params_.default_capture_radius_z;
    wp.slip_radius = src.slip_radius > 0.0 ? src.slip_radius : params_.default_slip_radius;
    wp.slip_radius_z = src.slip_radius_z > 0.0 ? src.slip_radius_z : params_.default_slip_radius_z;
    enu_waypoints.push_back(wp);

    RCLCPP_INFO(get_logger(),
                "Waypoint %zu: Lat %.6f, Lon %.6f, Depth %.2f, Speed %.1f RPM, "
                "Capture %.1f/%.1f m, Slip %.1f/%.1f m",
                i, gps.latitude, gps.longitude, gps.altitude, wp.speed, wp.capture_radius,
                wp.capture_radius_z, wp.slip_radius, wp.slip_radius_z);
  }

  blackboard_->set("mission_waypoints", enu_waypoints);
  RCLCPP_INFO(get_logger(), "Mission received: %zu waypoint(s).", msg->waypoints.size());
}

void BTHelmNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  blackboard_->set("last_odom_time", this->get_clock()->now().seconds());
  blackboard_->set("current_x", msg->pose.pose.position.x);
  blackboard_->set("current_y", msg->pose.pose.position.y);
  blackboard_->set("current_z", msg->pose.pose.position.z);
}

void BTHelmNode::tickTree() {
  blackboard_->set("current_time", this->get_clock()->now().seconds());
  tree_.tickOnce();
}

void BTHelmNode::checkBehaviorStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  auto active = static_cast<Behavior>(blackboard_->get<int>("active_behavior"));

  bool emergency = (active == Behavior::EMERGENCY_STOP || active == Behavior::EMERGENCY_SURFACE);
  stat.summary(emergency ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                         : diagnostic_msgs::msg::DiagnosticStatus::OK,
               toString(active));

  bool navigating =
      (active == Behavior::MISSION || active == Behavior::SURFACE || active == Behavior::HOME);
  auto wps = blackboard_->get<std::vector<Waypoint>>("active_waypoints");
  if (!navigating || wps.empty()) {
    return;
  }

  auto idx = blackboard_->get<size_t>("current_waypoint");
  if (idx >= wps.size()) {
    return;
  }

  double cx = blackboard_->get<double>("current_x");
  double cy = blackboard_->get<double>("current_y");
  double cz = blackboard_->get<double>("current_z");
  const auto& target = wps[idx];
  stat.addf("Waypoint", "%zu/%zu", idx + 1, wps.size());
  stat.addf("Horizontal Distance (m)", "%.1f",
            std::hypot(target.position.x - cx, target.position.y - cy));
  stat.addf("Vertical Distance (m)", "%.1f", std::abs(target.position.z - cz));
}

}  // namespace coug_helm

RCLCPP_COMPONENTS_REGISTER_NODE(coug_helm::BTHelmNode)
