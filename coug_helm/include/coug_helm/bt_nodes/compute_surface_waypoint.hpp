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

#include <behaviortree_cpp/bt_factory.h>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"
#include "coug_helm/utils/waypoint.hpp"

namespace coug_helm::bt_nodes {

class ComputeSurfaceWaypoint : public RosBtNode<BT::SyncActionNode> {
 public:
  ComputeSurfaceWaypoint(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  // --- Overrides ---
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("current_x"),
        BT::InputPort<double>("current_y"),
        BT::InputPort<double>("default_speed"),
        BT::InputPort<double>("surface_capture_radius"),
        BT::InputPort<double>("surface_capture_radius_z"),
        BT::InputPort<double>("surface_slip_radius"),
        BT::InputPort<double>("surface_slip_radius_z"),
        BT::OutputPort<std::vector<utils::Waypoint>>("surface_waypoint"),
    };
  }

  BT::NodeStatus tick() override {
    utils::Waypoint waypoint;
    waypoint.position.x = getInput<double>("current_x").value();
    waypoint.position.y = getInput<double>("current_y").value();
    waypoint.position.z = 0.0;
    waypoint.speed = getInput<double>("default_speed").value();

    waypoint.capture_radius = getInput<double>("surface_capture_radius").value();
    waypoint.capture_radius_z = getInput<double>("surface_capture_radius_z").value();
    waypoint.slip_radius = getInput<double>("surface_slip_radius").value();
    waypoint.slip_radius_z = getInput<double>("surface_slip_radius_z").value();

    RCLCPP_INFO(node_->get_logger(),
                "ComputeSurfaceWaypoint: surface set to (%.1f, %.1f), depth 0.",
                waypoint.position.x, waypoint.position.y);
    setOutput("surface_waypoint", std::vector<utils::Waypoint>{waypoint});
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
