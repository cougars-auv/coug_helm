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
 * @file wait.hpp
 * @brief BT recovery action that holds the AUV idle for a fixed duration.
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
 * @class Wait
 * @brief BT recovery action that holds the AUV idle for a fixed duration.
 */
class Wait : public RosBtNode<BT::StatefulActionNode> {
 public:
  Wait(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::StatefulActionNode>(name, config) {
    hsd_pub_ = node_->create_publisher<coug_interfaces::msg::ControlSetpoint>(
        config.blackboard->get<std::string>("hsd_topic"), rclcpp::SystemDefaultsQoS());
  }

  // --- Overrides ---
  static BT::PortsList providedPorts() { return {BT::InputPort<double>("wait_duration")}; }

  /**
   * @brief Records the start time and publishes a stop setpoint.
   * @return Always RUNNING.
   */
  BT::NodeStatus onStart() override {
    start_time_ = node_->now().seconds();
    RCLCPP_INFO(node_->get_logger(), "Wait: waiting %.1f s.",
                getInput<double>("wait_duration").value());
    publishStop();
    return BT::NodeStatus::RUNNING;
  }

  /**
   * @brief Holds the stop setpoint until the wait duration elapses.
   * @return RUNNING until the duration elapses, then SUCCESS.
   */
  BT::NodeStatus onRunning() override {
    publishStop();
    double duration = getInput<double>("wait_duration").value();
    if ((node_->now().seconds() - start_time_) >= duration) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

 private:
  /**
   * @brief Publishes a zero setpoint to stop the AUV.
   */
  void publishStop() {
    coug_interfaces::msg::ControlSetpoint msg;
    hsd_pub_->publish(msg);
  }

  // --- ROS Interfaces ---
  rclcpp::Publisher<coug_interfaces::msg::ControlSetpoint>::SharedPtr hsd_pub_;
  double start_time_{0.0};
};

}  // namespace coug_helm::bt_nodes
