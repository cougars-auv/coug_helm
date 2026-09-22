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
#include <coug_interfaces/msg/way_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class ComputeGoalPoses : public RosBtNode<BT::SyncActionNode> {
 public:
  ComputeGoalPoses(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints"),
        BT::InputPort<std::string>("map_frame"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto waypoints =
        getPortOrBlackboard<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints");
    if (waypoints.empty()) {
      RCLCPP_WARN(node_->get_logger(), "ComputeGoalPoses: no waypoints to convert.");
      return BT::NodeStatus::FAILURE;
    }

    std::vector<geometry_msgs::msg::PoseStamped> goals;
    goals.reserve(waypoints.size());

    const std::string map_frame = getPortOrBlackboard<std::string>("map_frame");
    const auto stamp = node_->now();

    double heading = 0.0;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (i + 1 < waypoints.size()) {
        heading = std::atan2(waypoints[i + 1].position.y - waypoints[i].position.y,
                             waypoints[i + 1].position.x - waypoints[i].position.x);
      }

      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, heading);

      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = map_frame;
      goal.header.stamp = stamp;
      goal.pose.position = waypoints[i].position;
      goal.pose.position.z = 0.0;
      goal.pose.orientation = tf2::toMsg(orientation);
      goals.push_back(goal);
    }

    RCLCPP_INFO(node_->get_logger(), "ComputeGoalPoses: %zu goal(s) in frame '%s'.", goals.size(),
                map_frame.c_str());
    setOutput("goals", goals);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
