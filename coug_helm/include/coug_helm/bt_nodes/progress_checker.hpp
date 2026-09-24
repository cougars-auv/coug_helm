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

#include <behaviortree_cpp/decorator_node.h>

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class ProgressChecker : public RosBtNode<BT::DecoratorNode> {
 public:
  ProgressChecker(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::DecoratorNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<double>("curr_x"),
        BT::InputPort<double>("curr_y"),
        BT::InputPort<double>("curr_z"),
        BT::InputPort<double>("progress_threshold"),
        BT::InputPort<double>("progress_timeout_sec"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto curr_x = getPortOrBlackboard<double>("curr_x");
    const auto curr_y = getPortOrBlackboard<double>("curr_y");
    const auto curr_z = getPortOrBlackboard<double>("curr_z");
    const auto threshold = getPortOrBlackboard<double>("progress_threshold");
    const auto timeout = getPortOrBlackboard<double>("progress_timeout_sec");
    const double now = node_->now().seconds();

    if (status() == BT::NodeStatus::IDLE) {
      seeded_ = false;
    }

    if (!seeded_ ||
        std::hypot(curr_x - baseline_x_, curr_y - baseline_y_, curr_z - baseline_z_) >= threshold) {
      baseline_x_ = curr_x;
      baseline_y_ = curr_y;
      baseline_z_ = curr_z;
      last_progress_time_ = now;
      seeded_ = true;
    } else if (timeout > 0.0 && now - last_progress_time_ > timeout) {
      RCLCPP_WARN(node_->get_logger(),
                  "ProgressChecker: no progress for %.1f s; triggering recovery.",
                  now - last_progress_time_);
      resetChild();
      return BT::NodeStatus::FAILURE;
    }

    setStatus(BT::NodeStatus::RUNNING);
    return child_node_->executeTick();
  }

 private:
  bool seeded_{false};
  double baseline_x_{};
  double baseline_y_{};
  double baseline_z_{};
  double last_progress_time_{};
};

}  // namespace coug_helm::bt_nodes
