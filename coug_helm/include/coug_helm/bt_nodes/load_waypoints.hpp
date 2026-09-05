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
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class LoadWaypoints : public RosBtNode<BT::SyncActionNode> {
 public:
  LoadWaypoints(std::string const& name, BT::NodeConfig const& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<std::vector<coug_interfaces::msg::WayPoint>>("pending_waypoints"),
        BT::OutputPort<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints"),
        BT::OutputPort<size_t>("current_waypoint"),
        BT::OutputPort<double>("prev_norm_dist"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    auto waypoints =
        getInput<std::vector<coug_interfaces::msg::WayPoint>>("pending_waypoints").value();
    RCLCPP_INFO(node_->get_logger(), "LoadWaypoints: loading %zu waypoint(s).", waypoints.size());
    setOutput("active_waypoints", waypoints);
    setOutput("current_waypoint", size_t{0});
    setOutput("prev_norm_dist", -1.0);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
