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
 * @file compute_home_waypoint.hpp
 * @brief BT action node that computes the home waypoint.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <coug_interfaces/msg/control_setpoint.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"
#include "coug_helm/utils/waypoint.hpp"

namespace coug_helm::bt_nodes {

/**
 * @class ComputeHomeWaypoint
 * @brief BT action node that computes the home waypoint.
 */
class ComputeHomeWaypoint : public RosBtNode<BT::SyncActionNode> {
 public:
  ComputeHomeWaypoint(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  // --- Overrides ---
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<utils::Waypoint>>("mission_waypoints"),
        BT::InputPort<double>("default_speed"),
        BT::InputPort<double>("home_capture_radius"),
        BT::InputPort<double>("home_capture_radius_z"),
        BT::InputPort<double>("home_slip_radius"),
        BT::InputPort<double>("home_slip_radius_z"),
        BT::OutputPort<std::vector<utils::Waypoint>>("home_waypoint"),
    };
  }

  /**
   * @brief Stages a home waypoint at mission_waypoints[0], depth 0.
   * @return SUCCESS if a mission exists, FAILURE otherwise.
   */
  BT::NodeStatus tick() override {
    auto mission = getInput<std::vector<utils::Waypoint>>("mission_waypoints").value();
    if (mission.empty()) {
      RCLCPP_WARN(node_->get_logger(), "ComputeHomeWaypoint: no mission loaded to home from.");
      return BT::NodeStatus::FAILURE;
    }
    utils::Waypoint home = mission[0];
    home.position.z = 0.0;
    home.mode = coug_interfaces::msg::ControlSetpoint::DEPTH;

    home.speed = getInput<double>("default_speed").value();

    home.capture_radius = getInput<double>("home_capture_radius").value();
    home.capture_radius_z = getInput<double>("home_capture_radius_z").value();
    home.slip_radius = getInput<double>("home_slip_radius").value();
    home.slip_radius_z = getInput<double>("home_slip_radius_z").value();

    RCLCPP_INFO(node_->get_logger(), "ComputeHomeWaypoint: home set to (%.1f, %.1f), depth 0.",
                home.position.x, home.position.y);
    setOutput("home_waypoint", std::vector<utils::Waypoint>{home});
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
