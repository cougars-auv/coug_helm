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
#include <cstddef>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class LoadNextGoal : public RosBtNode<BT::SyncActionNode> {
 public:
  LoadNextGoal(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints"),
        BT::InputPort<double>("current_x"),
        BT::InputPort<double>("current_y"),
        BT::InputPort<std::string>("map_frame"),
        BT::BidirectionalPort<size_t>("current_waypoint"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto waypoints =
        getPortOrBlackboard<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints");
    const size_t index = getPortOrBlackboard<size_t>("current_waypoint");

    if (index >= waypoints.size()) {
      RCLCPP_INFO(node_->get_logger(), "LoadNextGoal: all %zu waypoint(s) reached.",
                  waypoints.size());
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = getPortOrBlackboard<std::string>("map_frame");
    goal.header.stamp = node_->now();
    goal.pose.position = waypoints[index].position;
    goal.pose.position.z = 0.0;

    const double heading =
        std::atan2(goal.pose.position.y - getPortOrBlackboard<double>("current_y"),
                   goal.pose.position.x - getPortOrBlackboard<double>("current_x"));
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    goal.pose.orientation = tf2::toMsg(orientation);

    RCLCPP_INFO(node_->get_logger(), "LoadNextGoal: waypoint %zu of %zu at (%.1f, %.1f).",
                index + 1, waypoints.size(), goal.pose.position.x, goal.pose.position.y);
    setOutput("goal", goal);
    setOutput("current_waypoint", index + 1);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
