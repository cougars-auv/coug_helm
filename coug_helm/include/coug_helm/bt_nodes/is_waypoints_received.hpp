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

#include <coug_interfaces/msg/way_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class IsWaypointsReceived : public RosBtNode<BT::ConditionNode> {
 public:
  IsWaypointsReceived(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::ConditionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {BT::InputPort<std::vector<coug_interfaces::msg::WayPoint>>("mission_waypoints")};
  }

  auto tick() -> BT::NodeStatus override {
    if (!getInput<std::vector<coug_interfaces::msg::WayPoint>>("mission_waypoints")
             .value()
             .empty()) {
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(node_->get_logger(), "IsWaypointsReceived: no waypoints received.");
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace coug_helm::bt_nodes
