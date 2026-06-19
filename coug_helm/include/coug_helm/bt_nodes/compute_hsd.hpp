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
 * @file compute_hsd.hpp
 * @brief BT action node that computes heading, speed, and depth commands.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <vector>

namespace coug_helm::bt_nodes {

/**
 * @class ComputeHSD
 * @brief Computes heading, speed, and depth toward the current waypoint.
 */
class ComputeHSD : public BT::SyncActionNode {
 public:
  /**
   * @brief Constructor for ComputeHSD.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  ComputeHSD(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Point>>("waypoints"),
        BT::InputPort<std::vector<double>>("waypoint_speeds"),
        BT::InputPort<size_t>("waypoint_index"),
        BT::InputPort<double>("current_x"),
        BT::InputPort<double>("current_y"),
        BT::InputPort<double>("default_speed"),
        BT::OutputPort<double>("heading"),
        BT::OutputPort<double>("speed"),
        BT::OutputPort<double>("depth"),
        BT::OutputPort<uint8_t>("mode"),
    };
  }

  /**
   * @brief Computes and writes heading, speed, depth, and mode to the blackboard.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    auto wps = getInput<std::vector<geometry_msgs::msg::Point>>("waypoints").value();
    auto speeds = getInput<std::vector<double>>("waypoint_speeds").value();
    auto idx = getInput<size_t>("waypoint_index").value();
    double cx = getInput<double>("current_x").value();
    double cy = getInput<double>("current_y").value();

    if (wps.empty() || idx >= wps.size()) return BT::NodeStatus::FAILURE;
    const auto& target = wps[idx];
    double dx = target.x - cx;
    double dy = target.y - cy;
    double speed = (idx < speeds.size()) ? speeds[idx] : getInput<double>("default_speed").value();

    setOutput("heading", std::atan2(dy, dx) * 180.0 / M_PI);
    setOutput("speed", speed);
    setOutput("depth", target.z);
    setOutput("mode", target.z > 0.0 ? uint8_t{1} : uint8_t{0});

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
