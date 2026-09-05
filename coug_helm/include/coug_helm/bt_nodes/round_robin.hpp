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

#include <string>

namespace coug_helm::bt_nodes {

class RoundRobin : public BT::ControlNode {
 public:
  RoundRobin(std::string const& name, BT::NodeConfig const& config)
      : BT::ControlNode(name, config) {}

  static auto providedPorts() -> BT::PortsList { return {}; }

  auto tick() -> BT::NodeStatus override {
    size_t const num_children = children_nodes_.size();
    setStatus(BT::NodeStatus::RUNNING);

    while (num_failures_ < num_children) {
      BT::NodeStatus const child_status = children_nodes_[current_child_idx_]->executeTick();

      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          num_failures_ = 0;
          haltChildren();
          advance(num_children);
          return BT::NodeStatus::SUCCESS;
        case BT::NodeStatus::FAILURE:
          num_failures_++;
          advance(num_children);
          break;  // try the next child within this tick
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;
        default:
          throw BT::LogicError("Invalid status returned by child");
      }
    }

    haltChildren();
    num_failures_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  void halt() override {
    current_child_idx_ = 0;
    num_failures_ = 0;
    BT::ControlNode::halt();
  }

 private:
  void advance(size_t num_children) {
    current_child_idx_ = (current_child_idx_ + 1) % num_children;
  }

  size_t current_child_idx_{0};
  size_t num_failures_{0};
};

}  // namespace coug_helm::bt_nodes
