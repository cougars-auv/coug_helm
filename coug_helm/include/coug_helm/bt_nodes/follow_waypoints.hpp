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

#include <cmath>
#include <coug_interfaces/msg/control_setpoint.hpp>
#include <coug_interfaces/msg/way_point.hpp>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class FollowWaypoints : public RosBtNode<BT::StatefulActionNode> {
 public:
  FollowWaypoints(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::StatefulActionNode>(name, config) {
    hsd_pub_ = node_->create_publisher<coug_interfaces::msg::ControlSetpoint>(
        config.blackboard->get<std::string>("hsd_topic"), rclcpp::SystemDefaultsQoS());
  }

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints"),
        BT::InputPort<double>("current_x"),
        BT::InputPort<double>("current_y"),
        BT::InputPort<double>("current_z"),
        BT::BidirectionalPort<size_t>("current_waypoint"),
        BT::BidirectionalPort<double>("prev_norm_dist"),
    };
  }

  auto onStart() -> BT::NodeStatus override {
    auto waypoints =
        getInput<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints").value();
    auto waypoint_idx = getInput<size_t>("current_waypoint").value();
    if (!waypoints.empty() && waypoint_idx < waypoints.size()) {
      RCLCPP_INFO(node_->get_logger(), "FollowWaypoints: navigating %zu waypoint(s).",
                  waypoints.size());
    }
    return onRunning();
  }

  auto onRunning() -> BT::NodeStatus override {
    auto waypoints =
        getInput<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints").value();
    auto waypoint_idx = getInput<size_t>("current_waypoint").value();

    if (waypoints.empty() || waypoint_idx >= waypoints.size()) {
      publishStop();
      if (!waypoints.empty()) {
        RCLCPP_INFO(node_->get_logger(), "FollowWaypoints: completed waypoint navigation.");
      }
      return BT::NodeStatus::SUCCESS;
    }

    const double current_x = getInput<double>("current_x").value();
    const double current_y = getInput<double>("current_y").value();
    const double current_z = getInput<double>("current_z").value();
    const double prev_norm_dist = getInput<double>("prev_norm_dist").value();

    const auto& target = waypoints[waypoint_idx];
    publishHsd(target, current_x, current_y);
    const double horizontal_dist =
        std::hypot(target.position.x - current_x, target.position.y - current_y);
    const double vertical_dist = (target.mode == coug_interfaces::msg::WayPoint::ALTITUDE)
                                     ? 0.0
                                     : std::abs(target.position.z - current_z);

    const double norm_capture_dist = std::hypot(horizontal_dist / target.capture_radius,
                                                vertical_dist / target.capture_radius_z);
    const double norm_slip_dist =
        std::hypot(horizontal_dist / target.slip_radius, vertical_dist / target.slip_radius_z);

    const bool capture = norm_capture_dist < 1.0;
    const bool slip =
        (prev_norm_dist > 0.0 && norm_capture_dist > prev_norm_dist && norm_slip_dist < 1.0);

    if (capture || slip) {
      RCLCPP_INFO(node_->get_logger(), "FollowWaypoints: reached waypoint %zu/%zu (%s).",
                  waypoint_idx + 1, waypoints.size(), capture ? "capture" : "slip");
      setOutput("current_waypoint", waypoint_idx + 1);
      setOutput("prev_norm_dist", -1.0);  // new target, reset slip baseline
    } else {
      setOutput("prev_norm_dist", norm_capture_dist);
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { publishStop(); }

 private:
  void publishHsd(const coug_interfaces::msg::WayPoint& target, double current_x,
                  double current_y) {
    static constexpr double kRadToDeg = 180.0 / M_PI;

    const double delta_x = target.position.x - current_x;
    const double delta_y = target.position.y - current_y;

    coug_interfaces::msg::ControlSetpoint hsd_msg;
    hsd_msg.heading = std::atan2(delta_y, delta_x) * kRadToDeg;
    hsd_msg.speed_rpm = target.speed_rpm;
    hsd_msg.depth = target.position.z;
    hsd_msg.mode = target.mode;
    hsd_pub_->publish(hsd_msg);
  }

  void publishStop() {
    const coug_interfaces::msg::ControlSetpoint hsd_msg;
    hsd_pub_->publish(hsd_msg);
  }

  rclcpp::Publisher<coug_interfaces::msg::ControlSetpoint>::SharedPtr hsd_pub_;
};

}  // namespace coug_helm::bt_nodes
