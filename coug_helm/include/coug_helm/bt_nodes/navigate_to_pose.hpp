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

#include <nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace coug_helm::bt_nodes {

class NavigateToPose : public nav2_behavior_tree::NavigateToPoseAction {
 public:
  NavigateToPose(const std::string& name, const std::string& action_name,
                 const BT::NodeConfig& config)
      : NavigateToPoseAction(name, action_name, config) {}

  void on_tick() override {
    NavigateToPoseAction::on_tick();
    RCLCPP_INFO(node_->get_logger(), "%s: navigating to (%.1f, %.1f) m.",
                registrationName().c_str(), goal_.pose.pose.position.x, goal_.pose.pose.position.y);
  }

  auto on_success() -> BT::NodeStatus override {
    RCLCPP_INFO(node_->get_logger(), "%s: goal reached.", registrationName().c_str());
    return NavigateToPoseAction::on_success();
  }

  auto on_aborted() -> BT::NodeStatus override {
    std::string reason = "error code " + std::to_string(result_.result->error_code);
    if (!result_.result->error_msg.empty()) {
      reason += ", " + result_.result->error_msg;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: goal aborted (%s).", registrationName().c_str(),
                reason.c_str());
    return NavigateToPoseAction::on_aborted();
  }

  auto on_cancelled() -> BT::NodeStatus override {
    if (!halting_) {
      RCLCPP_WARN(node_->get_logger(), "%s: goal cancelled.", registrationName().c_str());
    }
    NavigateToPoseAction::on_cancelled();
    return BT::NodeStatus::FAILURE;
  }

  void halt() override {
    halting_ = true;
    NavigateToPoseAction::halt();
    halting_ = false;
  }

 private:
  bool halting_{false};
};

}  // namespace coug_helm::bt_nodes
