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

#include "coug_helm/bt_helm.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
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

using coug_interfaces::msg::WayPoint;
using utils::Behavior;
using utils::toString;

BtHelmNode::BtHelmNode(const rclcpp::NodeOptions& options)
    : Node("bt_helm_node", options), diagnostic_updater_(this) {
  param_listener_ = std::make_shared<bt_helm_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- Blackboard ---
  blackboard_ = BT::Blackboard::create();

  blackboard_->set("node", static_cast<rclcpp::Node*>(this));
  blackboard_->set("hsd_topic", params_.hsd_topic);
  blackboard_->set("arm_thruster_service", params_.arm_thruster_service);
  blackboard_->set("reset_localization_service", params_.reset_localization_service);

  blackboard_->set("pending_behavior", static_cast<int>(Behavior::kStop));
  blackboard_->set("active_behavior", static_cast<int>(Behavior::kStop));

  blackboard_->set("current_waypoint", size_t{0});
  blackboard_->set("prev_norm_dist", -1.0);

  blackboard_->set("active_waypoints", std::vector<WayPoint>{});
  blackboard_->set("mission_waypoints", std::vector<WayPoint>{});

  blackboard_->set("current_x", 0.0);
  blackboard_->set("current_y", 0.0);
  blackboard_->set("current_z", 0.0);
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

  waypoint_sub_ = create_subscription<coug_interfaces::msg::WayPointList>(
      params_.waypoint_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BtHelmNode::waypointCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BtHelmNode::odomCallback, this, std::placeholders::_1));

  start_srv_ = createBehaviorService(params_.start_service, Behavior::kMission, "Mission");
  stop_srv_ = createBehaviorService(params_.stop_service, Behavior::kStop, "Stop");
  surface_srv_ = createBehaviorService(params_.surface_service, Behavior::kSurface, "Surface");
  home_srv_ = createBehaviorService(params_.home_service, Behavior::kHome, "Home");
  emergency_stop_srv_ = createBehaviorService(params_.emergency_stop_service,
                                              Behavior::kEmergencyStop, "Emergency stop");
  emergency_surface_srv_ = createBehaviorService(params_.emergency_surface_service,
                                                 Behavior::kEmergencySurface, "Emergency surface");

  tick_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / params_.tick_rate_hz),
                                  std::bind(&BtHelmNode::tickTree, this));

  factory_.registerNodeType<bt_nodes::IsOdomHealthy>("IsOdomHealthy");
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

  if (params_.publish_groot2) {
    groot2_pub_ = std::make_unique<BT::Groot2Publisher>(tree_, params_.groot2_port);
    RCLCPP_INFO(get_logger(), "Groot2 Publisher: Port %ld", params_.groot2_port);
  }

  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/bt_helm_node");

    std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";

    std::string behavior_task = prefix + "Behavior Status";
    diagnostic_updater_.add(behavior_task, this, &BtHelmNode::checkBehaviorStatus);
  }

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void BtHelmNode::waypointCallback(const coug_interfaces::msg::WayPointList::SharedPtr msg) {
  if (msg->waypoints.empty()) {
    return;
  }

  std::vector<WayPoint> map_waypoints;
  for (size_t i = 0; i < msg->waypoints.size(); ++i) {
    const auto& src_waypoint = msg->waypoints[i];
    WayPoint waypoint;
    waypoint.position = src_waypoint.position;
    waypoint.mode = src_waypoint.mode;
    waypoint.speed_rpm = src_waypoint.speed_rpm;
    waypoint.capture_radius = src_waypoint.capture_radius > 0.0 ? src_waypoint.capture_radius
                                                                : params_.default_capture_radius;
    waypoint.capture_radius_z = src_waypoint.capture_radius_z > 0.0
                                    ? src_waypoint.capture_radius_z
                                    : params_.default_capture_radius_z;
    waypoint.slip_radius =
        src_waypoint.slip_radius > 0.0 ? src_waypoint.slip_radius : params_.default_slip_radius;
    waypoint.slip_radius_z = src_waypoint.slip_radius_z > 0.0 ? src_waypoint.slip_radius_z
                                                              : params_.default_slip_radius_z;
    map_waypoints.push_back(waypoint);

    RCLCPP_INFO(get_logger(),
                "Waypoint %zu: X %.2f, Y %.2f, Z %.2f, Speed %.1f RPM, "
                "Capture %.1f/%.1f m, Slip %.1f/%.1f m",
                i, waypoint.position.x, waypoint.position.y, waypoint.position.z,
                waypoint.speed_rpm, waypoint.capture_radius, waypoint.capture_radius_z,
                waypoint.slip_radius, waypoint.slip_radius_z);
  }

  blackboard_->set("mission_waypoints", map_waypoints);
  RCLCPP_INFO(get_logger(), "Mission received: %zu waypoint(s).", msg->waypoints.size());
}

void BtHelmNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  blackboard_->set("last_odom_time", this->get_clock()->now().seconds());
  blackboard_->set("current_x", msg->pose.pose.position.x);
  blackboard_->set("current_y", msg->pose.pose.position.y);
  blackboard_->set("current_z", msg->pose.pose.position.z);
}

rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr BtHelmNode::createBehaviorService(
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

void BtHelmNode::tickTree() {
  blackboard_->set("current_time", this->get_clock()->now().seconds());
  tree_.tickOnce();
}

void BtHelmNode::checkBehaviorStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  auto active = static_cast<Behavior>(blackboard_->get<int>("active_behavior"));

  bool emergency = (active == Behavior::kEmergencyStop || active == Behavior::kEmergencySurface);
  stat.summary(emergency ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                         : diagnostic_msgs::msg::DiagnosticStatus::OK,
               toString(active));

  bool navigating =
      (active == Behavior::kMission || active == Behavior::kSurface || active == Behavior::kHome);
  auto waypoints = blackboard_->get<std::vector<WayPoint>>("active_waypoints");
  if (!navigating || waypoints.empty()) {
    return;
  }

  auto waypoint_idx = blackboard_->get<size_t>("current_waypoint");
  if (waypoint_idx >= waypoints.size()) {
    return;
  }

  double current_x = blackboard_->get<double>("current_x");
  double current_y = blackboard_->get<double>("current_y");
  double current_z = blackboard_->get<double>("current_z");
  const auto& target = waypoints[waypoint_idx];
  stat.addf("Waypoint", "%zu/%zu", waypoint_idx + 1, waypoints.size());
  stat.addf("Horizontal Distance (m)", "%.1f",
            std::hypot(target.position.x - current_x, target.position.y - current_y));
  stat.addf("Vertical Distance (m)", "%.1f", std::abs(target.position.z - current_z));
}

}  // namespace coug_helm

RCLCPP_COMPONENTS_REGISTER_NODE(coug_helm::BtHelmNode)
