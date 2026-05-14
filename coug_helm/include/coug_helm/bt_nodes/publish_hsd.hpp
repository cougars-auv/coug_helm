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
 * @file publish_hsd.hpp
 * @brief BT action node that publishes heading, speed, and depth commands.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <coug_interfaces/msg/control_setpoint.hpp>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace coug_helm::bt_nodes {

/**
 * @class PublishHSD
 * @brief Publishes the computed heading, speed, and depth as a single ControlSetpoint message.
 */
class PublishHSD : public BT::SyncActionNode {
 public:
  /**
   * @brief Constructor for PublishHSD.
   * @param name The name of the node.
   * @param config The BT node configuration.
   * @param hsd_pub Publisher for the combined HSD setpoint.
   */
  PublishHSD(const std::string& name, const BT::NodeConfig& config,
             rclcpp::Publisher<coug_interfaces::msg::ControlSetpoint>::SharedPtr hsd_pub)
      : BT::SyncActionNode(name, config), hsd_pub_(std::move(hsd_pub)) {}

  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Publishes heading, speed, and depth from the blackboard as a ControlSetpoint.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    coug_interfaces::msg::ControlSetpoint msg;
    msg.heading = config().blackboard->get<double>("heading");
    msg.speed = config().blackboard->get<double>("speed");
    msg.depth = config().blackboard->get<double>("depth");
    msg.mode = config().blackboard->get<uint8_t>("mode");
    hsd_pub_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }

 private:
  // --- ROS Interfaces ---
  rclcpp::Publisher<coug_interfaces::msg::ControlSetpoint>::SharedPtr hsd_pub_;
};

}  // namespace coug_helm::bt_nodes
