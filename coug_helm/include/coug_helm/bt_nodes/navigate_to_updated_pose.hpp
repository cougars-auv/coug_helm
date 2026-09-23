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

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <string>

namespace coug_helm::bt_nodes {

class NavigateToUpdatedPose : public nav2_behavior_tree::NavigateToPoseAction {
 public:
  NavigateToUpdatedPose(const std::string& name, const std::string& action_name,
                        const BT::NodeConfig& config)
      : NavigateToPoseAction(name, action_name, config) {}

  static auto providedPorts() -> BT::PortsList {
    auto ports = NavigateToPoseAction::providedPorts();
    ports.insert(BT::InputPort<double>("goal_shift_threshold"));
    return ports;
  }

  void on_wait_for_result(
      std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> /*feedback*/) override {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) {
      return;
    }
    double shift_threshold = 0.0;
    getInputOrBlackboard("goal_shift_threshold", shift_threshold);
    const double shift = std::hypot(goal.pose.position.x - goal_.pose.pose.position.x,
                                    goal.pose.position.y - goal_.pose.pose.position.y);
    if (shift > shift_threshold) {
      RCLCPP_INFO(node_->get_logger(),
                  "NavigateToUpdatedPose: goal moved %.1f m; resending to (%.1f, %.1f) m.", shift,
                  goal.pose.position.x, goal.pose.position.y);
      goal_.pose = goal;
      goal_updated_ = true;
    }
  }

  auto on_cancelled() -> BT::NodeStatus override { return BT::NodeStatus::FAILURE; }
};

}  // namespace coug_helm::bt_nodes
