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
 * @file reset_waypoint_index.hpp
 * @brief BT action node that resets the waypoint index to 0.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace coug_helm::bt_nodes {

/**
 * @class ResetWaypointIndex
 * @brief Resets waypoint_index to 0 and clears prev_norm_dist to restart traversal from the
 *        beginning of the waypoint list.
 */
class ResetWaypointIndex : public BT::SyncActionNode {
 public:
  /**
   * @brief Constructor for ResetWaypointIndex.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  ResetWaypointIndex(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<size_t>("waypoint_index"),
        BT::OutputPort<double>("prev_norm_dist"),
    };
  }

  /**
   * @brief Sets waypoint_index to 0 and prev_norm_dist to -1.0.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    setOutput("waypoint_index", size_t{0});
    setOutput("prev_norm_dist", -1.0);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
