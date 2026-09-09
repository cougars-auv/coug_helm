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

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class Wait : public RosBtNode<BT::StatefulActionNode> {
 public:
  Wait(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::StatefulActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList { return {BT::InputPort<double>("wait_duration")}; }

  auto onStart() -> BT::NodeStatus override {
    start_time_ = node_->now().seconds();
    RCLCPP_INFO(node_->get_logger(), "Wait: waiting %.1f s.",
                getInput<double>("wait_duration").value());
    return BT::NodeStatus::RUNNING;
  }

  auto onRunning() -> BT::NodeStatus override {
    const double duration = getInput<double>("wait_duration").value();
    if ((node_->now().seconds() - start_time_) >= duration) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

 private:
  double start_time_{};
};

}  // namespace coug_helm::bt_nodes
