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

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("current_x"),          BT::InputPort<double>("current_y"),
        BT::InputPort<double>("current_z"),          BT::InputPort<double>("current_time"),
        BT::InputPort<double>("progress_threshold"), BT::InputPort<double>("progress_timeout"),
    };
  }

  BT::NodeStatus tick() override {
    double current_x = getInput<double>("current_x").value();
    double current_y = getInput<double>("current_y").value();
    double current_z = getInput<double>("current_z").value();
    double current_time = getInput<double>("current_time").value();
    double threshold = getInput<double>("progress_threshold").value();
    double timeout = getInput<double>("progress_timeout").value();

    // Re-seed the baseline on entry and whenever the agent advances by threshold.
    if (!seeded_ || std::hypot(current_x - baseline_x_, current_y - baseline_y_,
                               current_z - baseline_z_) >= threshold) {
      baseline_x_ = current_x;
      baseline_y_ = current_y;
      baseline_z_ = current_z;
      last_progress_time_ = current_time;
      seeded_ = true;
    } else if (timeout > 0.0 && (current_time - last_progress_time_) > timeout) {
      RCLCPP_WARN(node_->get_logger(),
                  "ProgressChecker: no progress for %.1f s; triggering recovery.",
                  current_time - last_progress_time_);
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
  double baseline_x_;
  double baseline_y_;
  double baseline_z_;
  double last_progress_time_;
};

}  // namespace coug_helm::bt_nodes
