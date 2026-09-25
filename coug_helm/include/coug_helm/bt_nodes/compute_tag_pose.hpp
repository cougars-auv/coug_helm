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

class ComputeTagPose : public RosBtNode<BT::SyncActionNode> {
 public:
  ComputeTagPose(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<int>("tag_id"),
        BT::InputPort<std::map<int, geometry_msgs::msg::Point>>("detected_tags"),
        BT::InputPort<double>("curr_x"),
        BT::InputPort<double>("curr_y"),
        BT::InputPort<std::string>("map_frame"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("tag_pose"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto tag_id = getInput<int>("tag_id").value();
    const auto tags =
        getPortOrBlackboard<std::map<int, geometry_msgs::msg::Point>>("detected_tags");
    const auto tag_it = tags.find(tag_id);
    if (tag_it == tags.end()) {
      RCLCPP_WARN(node_->get_logger(), "ComputeTagPose: tag %d not found.", tag_id);
      return BT::NodeStatus::FAILURE;
    }

    const auto& tag = tag_it->second;
    const double heading = std::atan2(tag.y - getPortOrBlackboard<double>("curr_y"),
                                      tag.x - getPortOrBlackboard<double>("curr_x"));

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = getPortOrBlackboard<std::string>("map_frame");
    pose.header.stamp = node_->now();
    pose.pose.position.x = tag.x;
    pose.pose.position.y = tag.y;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    pose.pose.orientation = tf2::toMsg(orientation);

    setOutput("tag_pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
