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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <string>

#include "coug_helm/bt_nodes/navigate_to_pose.hpp"

namespace coug_helm::bt_nodes {

class NavigateToTrackedPose : public NavigateToPose {
 public:
  NavigateToTrackedPose(const std::string& name, const std::string& action_name,
                        const BT::NodeConfig& config)
      : NavigateToPose(name, action_name, config) {}

  void on_wait_for_result(
      std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> /*feedback*/) override {
    geometry_msgs::msg::PoseStamped goal;
    if (getInput("goal", goal) && goal.pose.position != goal_.pose.pose.position) {
      goal_.pose = goal;
      goal_updated_ = true;
    }
  }
};

}  // namespace coug_helm::bt_nodes
