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
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class LoopWaypoints : public RosBtNode<BT::DecoratorNode> {
 public:
  LoopWaypoints(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::DecoratorNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints"),
        BT::BidirectionalPort<size_t>("waypoint_index"),
        BT::OutputPort<coug_interfaces::msg::WayPoint>("goal_waypoint"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto waypoints =
        getInput<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints").value();
    const size_t index = getInput<size_t>("waypoint_index").value();
    if (index >= waypoints.size()) {
      return BT::NodeStatus::SUCCESS;
    }

    const auto& waypoint = waypoints[index];
    if (child_node_->status() == BT::NodeStatus::IDLE) {
      RCLCPP_INFO(node_->get_logger(),
                  "LoopWaypoints: navigating to waypoint %zu of %zu at (%.1f, %.1f) m.", index + 1,
                  waypoints.size(), waypoint.position.x, waypoint.position.y);
    }
    setOutput("goal_waypoint", waypoint);

    setStatus(BT::NodeStatus::RUNNING);
    const BT::NodeStatus child_status = child_node_->executeTick();
    if (child_status == BT::NodeStatus::RUNNING) {
      return child_status;
    }
    resetChild();
    if (child_status == BT::NodeStatus::FAILURE) {
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "LoopWaypoints: reached waypoint %zu of %zu.", index + 1,
                waypoints.size());
    setOutput("waypoint_index", index + 1);
    return index + 1 >= waypoints.size() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
};

}  // namespace coug_helm::bt_nodes
