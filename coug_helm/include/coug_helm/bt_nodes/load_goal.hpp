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

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class LoadGoal : public RosBtNode<BT::SyncActionNode> {
 public:
  LoadGoal(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<coug_interfaces::msg::WayPoint>("goal_waypoint"),
        BT::InputPort<double>("curr_x"),
        BT::InputPort<double>("curr_y"),
        BT::InputPort<std::string>("map_frame"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
        BT::OutputPort<int>("goal_type"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto waypoint = getInput<coug_interfaces::msg::WayPoint>("goal_waypoint").value();
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = getPortOrBlackboard<std::string>("map_frame");
    goal.header.stamp = node_->now();
    goal.pose.position = waypoint.position;
    goal.pose.position.z = 0.0;

    const double heading = std::atan2(goal.pose.position.y - getPortOrBlackboard<double>("curr_y"),
                                      goal.pose.position.x - getPortOrBlackboard<double>("curr_x"));
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    goal.pose.orientation = tf2::toMsg(orientation);

    setOutput("goal_pose", goal);
    setOutput("goal_type", static_cast<int>(waypoint.type));
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
