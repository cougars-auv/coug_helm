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
 * @file stop.hpp
 * @brief BT action node that halts the AUV by publishing zero commands.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <coug_interfaces/msg/control_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

/**
 * @class Stop
 * @brief BT action node that halts the AUV by publishing zero commands.
 */
class Stop : public RosBtNode<BT::SyncActionNode> {
 public:
  Stop(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {
    hsd_pub_ = node_->create_publisher<coug_interfaces::msg::ControlSetpoint>(
        config.blackboard->get<std::string>("hsd_topic"), rclcpp::SystemDefaultsQoS());
  }

  // --- Overrides ---
  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Publishes a zero ControlSetpoint to stop the AUV.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    coug_interfaces::msg::ControlSetpoint msg;
    hsd_pub_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }

 private:
  // --- ROS Interfaces ---
  rclcpp::Publisher<coug_interfaces::msg::ControlSetpoint>::SharedPtr hsd_pub_;
};

}  // namespace coug_helm::bt_nodes
