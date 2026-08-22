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
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "coug_helm/bt_helm_parameters.hpp"
#include "coug_helm/utils/behavior_enums.hpp"
#include "coug_helm/utils/waypoint.hpp"

namespace coug_helm {

class BTHelmNode : public rclcpp::Node {
 public:
  explicit BTHelmNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void originCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  void waypointCallback(const coug_interfaces::msg::WayPointList::SharedPtr msg);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // --- Helpers ---
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr createBehaviorService(
      const std::string& service, utils::Behavior behavior, const std::string& label);

  void tickTree();

  // --- Diagnostics ---
  void checkBehaviorStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // --- Behavior Tree ---
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> groot2_pub_;
  BT::Blackboard::Ptr blackboard_;

  // --- ROS Interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  rclcpp::Subscription<coug_interfaces::msg::WayPointList>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr surface_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_surface_srv_;

  rclcpp::TimerBase::SharedPtr tick_timer_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<bt_helm_node::ParamListener> param_listener_;
  bt_helm_node::Params params_;

  // --- State ---
  GeographicLib::LocalCartesian local_cartesian_;
  bool origin_set_{false};
};

}  // namespace coug_helm
