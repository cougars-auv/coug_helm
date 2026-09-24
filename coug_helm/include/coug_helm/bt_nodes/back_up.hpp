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

#include <coug_interfaces/msg/control_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class BackUp : public RosBtNode<BT::StatefulActionNode> {
 public:
  BackUp(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::StatefulActionNode>(name, config) {
    hsd_pub_ = node_->create_publisher<coug_interfaces::msg::ControlSetpoint>(
        config.blackboard->get<std::string>("hsd_topic"), rclcpp::SystemDefaultsQoS());
  }

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<double>("backup_speed_rpm"),
        BT::InputPort<double>("backup_duration_sec"),
        BT::InputPort<double>("curr_heading_degrees"),
        BT::InputPort<double>("curr_z"),
    };
  }

  auto onStart() -> BT::NodeStatus override {
    start_time_ = node_->now().seconds();
    duration_ = getPortOrBlackboard<double>("backup_duration_sec");

    hsd_msg_.heading = getPortOrBlackboard<double>("curr_heading_degrees");
    hsd_msg_.speed_rpm = getPortOrBlackboard<double>("backup_speed_rpm");
    hsd_msg_.depth = getPortOrBlackboard<double>("curr_z");
    hsd_msg_.mode = coug_interfaces::msg::ControlSetpoint::DEPTH;

    RCLCPP_INFO(node_->get_logger(), "BackUp: reversing at %.0f RPM for %.1f s.",
                hsd_msg_.speed_rpm, duration_);
    return onRunning();
  }

  auto onRunning() -> BT::NodeStatus override {
    if (node_->now().seconds() - start_time_ >= duration_) {
      hsd_pub_->publish(coug_interfaces::msg::ControlSetpoint{});
      RCLCPP_INFO(node_->get_logger(), "BackUp: finished reversing.");
      return BT::NodeStatus::SUCCESS;
    }

    hsd_pub_->publish(hsd_msg_);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { hsd_pub_->publish(coug_interfaces::msg::ControlSetpoint{}); }

 private:
  rclcpp::Publisher<coug_interfaces::msg::ControlSetpoint>::SharedPtr hsd_pub_;
  coug_interfaces::msg::ControlSetpoint hsd_msg_;
  double start_time_{};
  double duration_{};
};

}  // namespace coug_helm::bt_nodes
