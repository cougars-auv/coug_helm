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
 * @file bt_helm.hpp
 * @brief ROS 2 node that drives a BehaviorTree.CPP waypoint mission.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <coug_interfaces/msg/way_point_list.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>

#include "coug_helm/bt_helm_parameters.hpp"

namespace coug_helm {

/**
 * @class BTHelmNode
 * @brief Behavior tree helm for AUV waypoint mission execution.
 */
class BTHelmNode : public rclcpp::Node {
 public:
  /**
   * @brief BTHelmNode constructor.
   * @param options The node options.
   */
  explicit BTHelmNode(const rclcpp::NodeOptions& options);

 private:
  // --- Logic ---
  /**
   * @brief Callback for the shared GPS origin fix. Sets the ENU projection origin.
   * @param msg The incoming NavSatFix message.
   */
  void originCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /**
   * @brief Callback for receiving a new waypoint mission. Projects GPS waypoints to ENU.
   * @param msg The incoming WayPointList message.
   */
  void waypointCallback(const coug_interfaces::msg::WayPointList::SharedPtr msg);

  /**
   * @brief Callback for odometry updates. Updates position on the blackboard.
   * @param msg The incoming Odometry message.
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Timer callback that ticks the behavior tree once per cycle.
   */
  void tickTree();

  // --- Diagnostics ---
  /**
   * @brief Diagnostic task to report mission progress and waypoint status.
   * @param stat The diagnostic status wrapper to update.
   */
  void checkMissionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @brief Diagnostic task to report odometry health and freshness.
   * @param stat The diagnostic status wrapper to update.
   */
  void checkOdometryStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // --- Behavior Tree ---
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> groot2_pub_;
  BT::Blackboard::Ptr blackboard_;

  GeographicLib::LocalCartesian local_cartesian_;
  bool origin_set_{false};

  // --- ROS Interfaces ---
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  rclcpp::Subscription<coug_interfaces::msg::WayPointList>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr tick_timer_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<bt_helm_node::ParamListener> param_listener_;
  bt_helm_node::Params params_;
};

}  // namespace coug_helm
