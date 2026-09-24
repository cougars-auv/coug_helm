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
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class ComputeApproachPose : public RosBtNode<BT::SyncActionNode> {
 public:
  ComputeApproachPose(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<int>("tag_id"),
        BT::InputPort<std::map<int, geometry_msgs::msg::Point>>("detected_tags"),
        BT::InputPort<double>("curr_x"),
        BT::InputPort<double>("curr_y"),
        BT::InputPort<std::string>("map_frame"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("approach_pose"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto tag_id = getInput<int>("tag_id").value();
    const auto tags =
        getPortOrBlackboard<std::map<int, geometry_msgs::msg::Point>>("detected_tags");
    const auto tag_it = tags.find(tag_id);
    if (tag_it == tags.end()) {
      RCLCPP_WARN(node_->get_logger(),
                  "ComputeApproachPose: tag %d not found; nothing to approach.", tag_id);
      return BT::NodeStatus::FAILURE;
    }

    const auto& tag = tag_it->second;
    const auto curr_x = getPortOrBlackboard<double>("curr_x");
    const auto curr_y = getPortOrBlackboard<double>("curr_y");
    const double dx = tag.x - curr_x;
    const double dy = tag.y - curr_y;

    geometry_msgs::msg::PoseStamped approach;
    approach.header.frame_id = getPortOrBlackboard<std::string>("map_frame");
    approach.header.stamp = node_->now();
    approach.pose.position.x = tag.x;
    approach.pose.position.y = tag.y;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, std::atan2(dy, dx));
    approach.pose.orientation = tf2::toMsg(orientation);

    RCLCPP_DEBUG(node_->get_logger(), "ComputeApproachPose: approaching tag %d at (%.1f, %.1f) m.",
                 tag_id, approach.pose.position.x, approach.pose.position.y);
    setOutput("approach_pose", approach);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
