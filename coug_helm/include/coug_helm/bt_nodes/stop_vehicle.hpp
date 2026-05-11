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
 * @file stop_vehicle.hpp
 * @brief BT action node that halts the vehicle by publishing zero commands.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

namespace coug_helm::bt_nodes {

/**
 * @class StopVehicle
 * @brief Publishes zero heading, speed, and depth commands to halt the vehicle.
 */
class StopVehicle : public BT::SyncActionNode {
 public:
  /**
   * @brief Constructor for StopVehicle.
   * @param name The name of the node.
   * @param config The BT node configuration.
   * @param heading_pub Publisher for the heading command.
   * @param speed_pub Publisher for the speed command.
   * @param depth_pub Publisher for the depth command.
   */
  StopVehicle(const std::string& name, const BT::NodeConfig& config,
              rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub,
              rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub,
              rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub)
      : BT::SyncActionNode(name, config),
        heading_pub_(std::move(heading_pub)),
        speed_pub_(std::move(speed_pub)),
        depth_pub_(std::move(depth_pub)) {}

  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Publishes zero commands to all HSD topics.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    std_msgs::msg::Float64 msg;
    msg.data = 0.0;
    heading_pub_->publish(msg);
    speed_pub_->publish(msg);
    depth_pub_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }

 private:
  // --- ROS Interfaces ---
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
};

}  // namespace coug_helm::bt_nodes
