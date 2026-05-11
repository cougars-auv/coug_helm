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
 * @date May 2026
 */

#include "coug_helm/bt_helm.hpp"

#include <GeographicLib/LocalCartesian.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

#include "coug_helm/bt_nodes/advance_if_reached.hpp"
#include "coug_helm/bt_nodes/check_has_mission.hpp"
#include "coug_helm/bt_nodes/check_mission_complete.hpp"
#include "coug_helm/bt_nodes/check_odom_healthy.hpp"
#include "coug_helm/bt_nodes/compute_hsd.hpp"
#include "coug_helm/bt_nodes/publish_hsd.hpp"
#include "coug_helm/bt_nodes/stop_vehicle.hpp"

namespace coug_helm {

BTHelmNode::BTHelmNode(const rclcpp::NodeOptions& options)
    : Node("bt_helm_node", options), diagnostic_updater_(this) {
  RCLCPP_INFO(get_logger(), "Starting Mission Helm Node...");

  param_listener_ = std::make_shared<bt_helm_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- Blackboard ---
  blackboard_ = BT::Blackboard::create();
  blackboard_->set("waypoints", std::vector<geometry_msgs::msg::Point>{});
  blackboard_->set("waypoint_index", size_t{0});
  blackboard_->set("previous_distance", -1.0);
  blackboard_->set("current_x", 0.0);
  blackboard_->set("current_y", 0.0);
  blackboard_->set("last_odom_time", 0.0);
  blackboard_->set("current_time", 0.0);
  blackboard_->set("heading", 0.0);
  blackboard_->set("speed", 0.0);
  blackboard_->set("depth", 0.0);
  blackboard_->set("capture_radius", params_.capture_radius);
  blackboard_->set("slip_radius", params_.slip_radius);
  blackboard_->set("desired_speed_rpm", params_.desired_speed_rpm);
  blackboard_->set("odom_timeout_sec", params_.odom_timeout_sec);

  // --- ROS Interfaces ---
  heading_pub_ =
      create_publisher<std_msgs::msg::Float64>(params_.heading_topic, rclcpp::SystemDefaultsQoS());
  speed_pub_ =
      create_publisher<std_msgs::msg::Float64>(params_.speed_topic, rclcpp::SystemDefaultsQoS());
  depth_pub_ =
      create_publisher<std_msgs::msg::Float64>(params_.depth_topic, rclcpp::SystemDefaultsQoS());

  origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      params_.origin_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BTHelmNode::originCallback, this, std::placeholders::_1));

  waypoint_sub_ = create_subscription<coug_interfaces::msg::WayPointList>(
      params_.waypoint_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BTHelmNode::waypointCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BTHelmNode::odomCallback, this, std::placeholders::_1));

  // --- BT Node Registration ---
  factory_.registerNodeType<bt_nodes::CheckOdomHealthy>("CheckOdomHealthy");
  factory_.registerNodeType<bt_nodes::CheckHasMission>("CheckHasMission");
  factory_.registerNodeType<bt_nodes::CheckMissionComplete>("CheckMissionComplete");
  factory_.registerNodeType<bt_nodes::ComputeHSD>("ComputeHSD");
  factory_.registerNodeType<bt_nodes::AdvanceIfReached>("AdvanceIfReached");

  factory_.registerBuilder<bt_nodes::PublishHSD>(
      "PublishHSD", [h = heading_pub_, sp = speed_pub_, d = depth_pub_](
                        const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<bt_nodes::PublishHSD>(name, config, h, sp, d);
      });

  factory_.registerBuilder<bt_nodes::StopVehicle>(
      "StopVehicle", [h = heading_pub_, sp = speed_pub_, d = depth_pub_](
                         const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<bt_nodes::StopVehicle>(name, config, h, sp, d);
      });

  std::string pkg_share = ament_index_cpp::get_package_share_directory("coug_helm");
  tree_ = factory_.createTreeFromFile(pkg_share + "/trees/bt_helm_tree.xml", blackboard_);

  // --- Groot2 Publisher ---
  if (params_.publish_groot2) {
    groot2_pub_ = std::make_unique<BT::Groot2Publisher>(tree_, params_.groot2_port);
    RCLCPP_INFO(get_logger(), "Groot2 Publisher: Port %ld", params_.groot2_port);
  }

  // --- Tick Timer ---
  tick_timer_ = create_wall_timer(std::chrono::milliseconds(params_.tick_rate_ms),
                                  std::bind(&BTHelmNode::tickTree, this));

  // --- ROS Diagnostics ---
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/bt_helm_node");

    std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";
    diagnostic_updater_.add(prefix + "Mission Status", this, &BTHelmNode::checkMissionStatus);
    diagnostic_updater_.add(prefix + "Odometry Link", this, &BTHelmNode::checkOdometryStatus);
  }

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for mission...");
}

void BTHelmNode::originCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  if (!origin_set_ && msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    local_cartesian_.Reset(msg->latitude, msg->longitude, msg->altitude);
    origin_set_ = true;
    RCLCPP_INFO(get_logger(), "GPS Origin Set: Lat %.6f, Lon %.6f, Alt %.2f", msg->latitude,
                msg->longitude, msg->altitude);
  }
}

void BTHelmNode::waypointCallback(const coug_interfaces::msg::WayPointList::SharedPtr msg) {
  blackboard_->set("waypoint_index", size_t{0});
  blackboard_->set("previous_distance", -1.0);

  if (msg->waypoints.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty mission. Stopping.");
    blackboard_->set("waypoints", std::vector<geometry_msgs::msg::Point>{});
    return;
  }

  if (!origin_set_) {
    RCLCPP_WARN(get_logger(), "Origin not set. Dropping mission.");
    blackboard_->set("waypoints", std::vector<geometry_msgs::msg::Point>{});
    return;
  }

  std::vector<geometry_msgs::msg::Point> enu_waypoints;
  for (size_t i = 0; i < msg->waypoints.size(); ++i) {
    const auto& gps = msg->waypoints[i].position;
    geometry_msgs::msg::Point p;
    double dummy_z;
    local_cartesian_.Forward(gps.latitude, gps.longitude, 0.0, p.x, p.y, dummy_z);
    p.z = gps.altitude;  // altitude field carries depth, z-up
    enu_waypoints.push_back(p);
    RCLCPP_INFO(get_logger(), "Waypoint %zu: Lat %.6f, Lon %.6f, Depth %.2f", i, gps.latitude,
                gps.longitude, gps.altitude);
  }

  blackboard_->set("waypoints", enu_waypoints);
  RCLCPP_INFO(get_logger(), "Mission Received: %zu waypoint(s).", msg->waypoints.size());
}

void BTHelmNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  blackboard_->set("last_odom_time", this->get_clock()->now().seconds());
  blackboard_->set("current_x", msg->pose.pose.position.x);
  blackboard_->set("current_y", msg->pose.pose.position.y);
}

void BTHelmNode::tickTree() {
  blackboard_->set("current_time", this->get_clock()->now().seconds());
  tree_.tickOnce();
}

void BTHelmNode::checkMissionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  auto wps = blackboard_->get<std::vector<geometry_msgs::msg::Point>>("waypoints");
  auto idx = blackboard_->get<size_t>("waypoint_index");
  double prev_dist = blackboard_->get<double>("previous_distance");

  if (wps.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No mission received.");
  } else if (idx >= wps.size()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Mission complete.");
  } else {
    stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Navigating to waypoint " + std::to_string(idx + 1) + "/" + std::to_string(wps.size()));
    stat.add("Distance to Target (m)", prev_dist);
  }
}

void BTHelmNode::checkOdometryStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  double last_odom = blackboard_->get<double>("last_odom_time");
  double odom_timeout = blackboard_->get<double>("odom_timeout_sec");

  if (last_odom == 0.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No odometry received.");
    return;
  }
  double time_since = this->get_clock()->now().seconds() - last_odom;
  stat.add("Time Since Last (s)", time_since);
  if (time_since > odom_timeout) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Odometry link lost.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Odometry link online.");
  }
}

}  // namespace coug_helm

RCLCPP_COMPONENTS_REGISTER_NODE(coug_helm::BTHelmNode)
