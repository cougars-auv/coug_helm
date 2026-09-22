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
#include <behaviortree_cpp/decorators/loop_node.h>

#include <cmath>
#include <coug_interfaces/msg/way_point.hpp>
#include <deque>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class LoadSearchPoses : public RosBtNode<BT::SyncActionNode> {
 public:
  LoadSearchPoses(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<coug_interfaces::msg::WayPoint>("goal_waypoint"),
        BT::InputPort<std::string>("map_frame"),
        BT::OutputPort<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("search_poses"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto waypoint = getInput<coug_interfaces::msg::WayPoint>("goal_waypoint").value();

    auto search_poses = std::make_shared<std::deque<geometry_msgs::msg::PoseStamped>>();
    geometry_msgs::msg::Point search_from = waypoint.position;
    for (const auto& subwaypoint : waypoint.subwaypoints) {
      search_poses->push_back(makePose(search_from, subwaypoint));
      search_from = subwaypoint;
    }
    RCLCPP_INFO(node_->get_logger(), "LoadSearchPoses: loading %zu search pose(s).",
                search_poses->size());

    setOutput("search_poses", search_poses);
    return BT::NodeStatus::SUCCESS;
  }

 private:
  [[nodiscard]] auto makePose(const geometry_msgs::msg::Point& from,
                              const geometry_msgs::msg::Point& to) const
      -> geometry_msgs::msg::PoseStamped {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = getPortOrBlackboard<std::string>("map_frame");
    pose.header.stamp = node_->now();
    pose.pose.position = to;
    pose.pose.position.z = 0.0;

    const double heading = std::atan2(to.y - from.y, to.x - from.x);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    pose.pose.orientation = tf2::toMsg(orientation);
    return pose;
  }
};

}  // namespace coug_helm::bt_nodes
