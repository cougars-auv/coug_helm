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
 * @file advance_if_reached.hpp
 * @brief BT action node that advances the waypoint index when a target is reached.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <vector>

namespace coug_helm::bt_nodes {

/**
 * @class AdvanceIfReached
 * @brief Advances to the next waypoint if the capture or slip radius is met.
 */
class AdvanceIfReached : public BT::SyncActionNode {
 public:
  /**
   * @brief Constructor for AdvanceIfReached.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  AdvanceIfReached(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Checks distance to the current waypoint and advances the index if reached.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    auto wps = config().blackboard->get<std::vector<geometry_msgs::msg::Point>>("waypoints");
    auto idx = config().blackboard->get<size_t>("waypoint_index");
    double cx = config().blackboard->get<double>("current_x");
    double cy = config().blackboard->get<double>("current_y");
    double prev_dist = config().blackboard->get<double>("previous_distance");
    double cap_r = config().blackboard->get<double>("capture_radius");
    double slip_r = config().blackboard->get<double>("slip_radius");

    const auto& target = wps[idx];
    double dx = target.x - cx;
    double dy = target.y - cy;
    double dist = std::hypot(dx, dy);

    bool capture = dist < cap_r;
    bool slip = (prev_dist > 0.0 && dist > prev_dist && dist < slip_r);

    if (capture || slip) {
      config().blackboard->set("waypoint_index", idx + 1);
      config().blackboard->set("previous_distance", -1.0);
    } else {
      config().blackboard->set("previous_distance", dist);
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
