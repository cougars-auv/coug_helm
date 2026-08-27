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

#include <behaviortree_cpp/control_node.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class RecoveryNode : public RosBtNode<BT::ControlNode> {
 public:
  RecoveryNode(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::ControlNode>(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<int>("number_of_retries", 1, "Number of retries")};
  }

  BT::NodeStatus tick() override {
    const int number_of_retries = getInput<int>("number_of_retries").value();
    const size_t children_count = children_nodes_.size();

    if (children_count != 2) {
      throw BT::LogicError("RecoveryNode requires exactly 2 children");
    }

    setStatus(BT::NodeStatus::RUNNING);

    while (current_child_idx_ < children_count) {
      const BT::NodeStatus child_status = children_nodes_[current_child_idx_]->executeTick();

      if (current_child_idx_ == 0) {  // main behavior
        switch (child_status) {
          case BT::NodeStatus::SUCCESS:
            if (retry_count_ > 0) {
              RCLCPP_INFO(node_->get_logger(), "RecoveryNode: recovered after %d retry(ies).",
                          retry_count_);
            }
            haltChildren();
            current_child_idx_ = 0;
            retry_count_ = 0;
            return BT::NodeStatus::SUCCESS;
          case BT::NodeStatus::FAILURE:
            if (retry_count_ < number_of_retries) {
              haltChildren();
              current_child_idx_ = 1;
              RCLCPP_WARN(node_->get_logger(),
                          "RecoveryNode: behavior failed; running recovery (%d/%d).",
                          retry_count_ + 1, number_of_retries);
              break;
            }
            haltChildren();
            current_child_idx_ = 0;
            retry_count_ = 0;
            RCLCPP_ERROR(node_->get_logger(), "RecoveryNode: retries exhausted (%d).",
                         number_of_retries);
            return BT::NodeStatus::FAILURE;
          case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;
          default:
            throw BT::LogicError("Invalid status returned by child");
        }
      } else {  // recovery behavior
        switch (child_status) {
          case BT::NodeStatus::SUCCESS:
            haltChildren();
            retry_count_++;
            current_child_idx_ = 0;
            break;
          case BT::NodeStatus::FAILURE:
            haltChildren();
            current_child_idx_ = 0;
            retry_count_ = 0;
            return BT::NodeStatus::FAILURE;
          case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;
          default:
            throw BT::LogicError("Invalid status returned by child");
        }
      }
    }

    haltChildren();
    current_child_idx_ = 0;
    retry_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  void halt() override {
    current_child_idx_ = 0;
    retry_count_ = 0;
    BT::ControlNode::halt();
  }

 private:
  size_t current_child_idx_{0};
  int retry_count_{0};
};

}  // namespace coug_helm::bt_nodes
