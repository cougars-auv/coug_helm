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

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Point>>("waypoints"),
        BT::InputPort<double>("current_x"),
        BT::InputPort<double>("current_y"),
        BT::InputPort<double>("current_z"),
        BT::InputPort<std::vector<double>>("capture_radius"),
        BT::InputPort<std::vector<double>>("slip_radius"),
        BT::BidirectionalPort<size_t>("waypoint_index"),
        BT::BidirectionalPort<double>("prev_norm_dist"),
    };
  }

  /**
   * @brief Checks distance to the current waypoint and advances the index if reached.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    auto wps = getInput<std::vector<geometry_msgs::msg::Point>>("waypoints").value();
    auto idx = getInput<size_t>("waypoint_index").value();
    double cx = getInput<double>("current_x").value();
    double cy = getInput<double>("current_y").value();
    double cz = getInput<double>("current_z").value();
    double prev_dist = getInput<double>("prev_norm_dist").value();
    auto cap_r = getInput<std::vector<double>>("capture_radius").value();
    auto slip_r = getInput<std::vector<double>>("slip_radius").value();

    if (wps.empty() || idx >= wps.size()) return BT::NodeStatus::FAILURE;
    const auto& target = wps[idx];
    double h_dist = std::hypot(target.x - cx, target.y - cy);
    double v_dist = (target.z > 0.0) ? 0.0 : std::abs(target.z - cz);

    // Normalized ellipsoidal distance: < 1.0 means inside the ellipse
    double norm_cap = std::hypot(h_dist / cap_r[0], v_dist / cap_r[1]);
    double norm_slip = std::hypot(h_dist / slip_r[0], v_dist / slip_r[1]);

    bool capture = norm_cap < 1.0;
    bool slip = (prev_dist > 0.0 && norm_cap > prev_dist && norm_slip < 1.0);

    if (capture || slip) {
      setOutput("waypoint_index", idx + 1);
      setOutput("prev_norm_dist", -1.0);
    } else {
      setOutput("prev_norm_dist", norm_cap);
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
