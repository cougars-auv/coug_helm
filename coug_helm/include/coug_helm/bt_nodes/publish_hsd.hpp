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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

namespace coug_helm::bt_nodes {

/**
 * @class PublishHSD
 * @brief Publishes the computed heading, speed, and depth commands to ROS topics.
 */
class PublishHSD : public BT::SyncActionNode {
 public:
  /**
   * @brief Constructor for PublishHSD.
   * @param name The name of the node.
   * @param config The BT node configuration.
   * @param heading_pub Publisher for the heading command.
   * @param speed_pub Publisher for the speed command.
   * @param depth_pub Publisher for the depth command.
   */
  PublishHSD(const std::string& name, const BT::NodeConfig& config,
             rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub,
             rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub,
             rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub)
      : BT::SyncActionNode(name, config),
        heading_pub_(std::move(heading_pub)),
        speed_pub_(std::move(speed_pub)),
        depth_pub_(std::move(depth_pub)) {}

  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Publishes heading, speed, and depth from the blackboard.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    std_msgs::msg::Float64 msg;

    msg.data = config().blackboard->get<double>("heading");
    heading_pub_->publish(msg);

    msg.data = config().blackboard->get<double>("speed");
    speed_pub_->publish(msg);

    msg.data = config().blackboard->get<double>("depth");
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
