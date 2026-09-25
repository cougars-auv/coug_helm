// Copyright 2026 BYU FROST Lab
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

#include <coug_interfaces/msg/way_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class ComputeHomeWaypoint : public RosBtNode<BT::SyncActionNode> {
 public:
  ComputeHomeWaypoint(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<std::vector<coug_interfaces::msg::WayPoint>>("mission_waypoints"),
        BT::InputPort<double>("default_speed_rpm"),
        BT::InputPort<double>("home_capture_radius"),
        BT::InputPort<double>("home_capture_radius_z"),
        BT::InputPort<double>("home_slip_radius"),
        BT::InputPort<double>("home_slip_radius_z"),
        BT::OutputPort<std::vector<coug_interfaces::msg::WayPoint>>("home_waypoint"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    auto waypoints =
        getPortOrBlackboard<std::vector<coug_interfaces::msg::WayPoint>>("mission_waypoints");
    if (waypoints.empty()) {
      RCLCPP_WARN(node_->get_logger(), "ComputeHomeWaypoint: no mission waypoints to home to.");
      return BT::NodeStatus::FAILURE;
    }
    coug_interfaces::msg::WayPoint home;
    home.position = waypoints[0].position;
    home.position.z = 0.0;

    home.speed_rpm = getPortOrBlackboard<double>("default_speed_rpm");
    home.capture_radius = getPortOrBlackboard<double>("home_capture_radius");
    home.capture_radius_z = getPortOrBlackboard<double>("home_capture_radius_z");
    home.slip_radius = getPortOrBlackboard<double>("home_slip_radius");
    home.slip_radius_z = getPortOrBlackboard<double>("home_slip_radius_z");

    home.mode = coug_interfaces::msg::WayPoint::DEPTH;
    home.type = coug_interfaces::msg::WayPoint::GPS;

    RCLCPP_INFO(node_->get_logger(),
                "ComputeHomeWaypoint: home set to (%.1f, %.1f) m at the surface.", home.position.x,
                home.position.y);
    setOutput("home_waypoint", std::vector<coug_interfaces::msg::WayPoint>{home});
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
