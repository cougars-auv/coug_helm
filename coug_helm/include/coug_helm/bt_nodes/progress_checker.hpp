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
 * @file progress_checker.hpp
 * @brief BT decorator that fails its child when the vehicle stops making progress.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <behaviortree_cpp/decorator_node.h>

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

/**
 * @class ProgressChecker
 * @brief Ticks its child, but FAILS if the vehicle hasn't advanced.
 */
class ProgressChecker : public RosBtNode<BT::DecoratorNode> {
 public:
  ProgressChecker(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::DecoratorNode>(name, config) {}

  // --- Overrides ---
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("current_x"),          BT::InputPort<double>("current_y"),
        BT::InputPort<double>("current_z"),          BT::InputPort<double>("current_time"),
        BT::InputPort<double>("progress_threshold"), BT::InputPort<double>("progress_timeout"),
    };
  }

  /**
   * @brief Fails if the vehicle hasn't moved progress_threshold (m) within progress_timeout (s).
   * @return The child's status while progressing, FAILURE once stalled.
   */
  BT::NodeStatus tick() override {
    double cx = getInput<double>("current_x").value();
    double cy = getInput<double>("current_y").value();
    double cz = getInput<double>("current_z").value();
    double now = getInput<double>("current_time").value();
    double threshold = getInput<double>("progress_threshold").value();
    double timeout = getInput<double>("progress_timeout").value();

    // Re-seed the baseline on entry and whenever the vehicle advances by threshold.
    if (!seeded_ || std::hypot(cx - base_x_, cy - base_y_, cz - base_z_) >= threshold) {
      base_x_ = cx;
      base_y_ = cy;
      base_z_ = cz;
      last_progress_time_ = now;
      seeded_ = true;
    } else if (timeout > 0.0 && (now - last_progress_time_) > timeout) {
      RCLCPP_WARN(node_->get_logger(),
                  "ProgressChecker: no progress for %.1f s; triggering recovery.",
                  now - last_progress_time_);
      resetChild();
      return BT::NodeStatus::FAILURE;
    }

    setStatus(BT::NodeStatus::RUNNING);
    return child_node_->executeTick();
  }

  void halt() override {
    seeded_ = false;  // re-seed baseline on (re-)entry, e.g. after a recovery
    BT::DecoratorNode::halt();
  }

 private:
  bool seeded_{false};
  double base_x_{0.0};
  double base_y_{0.0};
  double base_z_{0.0};
  double last_progress_time_{0.0};
};

}  // namespace coug_helm::bt_nodes
