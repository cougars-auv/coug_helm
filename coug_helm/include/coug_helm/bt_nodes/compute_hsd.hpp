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

  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Computes and writes heading, speed, depth, and mode to the blackboard.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    auto wps = config().blackboard->get<std::vector<geometry_msgs::msg::Point>>("waypoints");
    auto idx = config().blackboard->get<size_t>("waypoint_index");
    double cx = config().blackboard->get<double>("current_x");
    double cy = config().blackboard->get<double>("current_y");
    double speed_rpm = config().blackboard->get<double>("desired_speed_rpm");

    const auto& target = wps[idx];
    double dx = target.x - cx;
    double dy = target.y - cy;

    config().blackboard->set("heading", std::atan2(dy, dx) * 180.0 / M_PI);
    config().blackboard->set("speed", speed_rpm);
    config().blackboard->set("depth", target.z);
    config().blackboard->set("mode", target.z > 0.0 ? uint8_t{1} : uint8_t{0});

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
