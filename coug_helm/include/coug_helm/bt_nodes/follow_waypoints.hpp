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

/**
 * @file follow_waypoints.hpp
 * @brief BT action node that steers the AUV through a waypoint list.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <cmath>
#include <coug_interfaces/msg/control_setpoint.hpp>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"
#include "coug_helm/utils/waypoint.hpp"

namespace coug_helm::bt_nodes {

/**
 * @class FollowWaypoints
 * @brief BT action node that steers the AUV through a waypoint list.
 */
class FollowWaypoints : public RosBtNode<BT::StatefulActionNode> {
 public:
  FollowWaypoints(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::StatefulActionNode>(name, config) {
    hsd_pub_ = node_->create_publisher<coug_interfaces::msg::ControlSetpoint>(
        config.blackboard->get<std::string>("hsd_topic"), rclcpp::SystemDefaultsQoS());
  }

  // --- Overrides ---
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<utils::Waypoint>>("active_waypoints"),
        BT::InputPort<double>("current_x"),
        BT::InputPort<double>("current_y"),
        BT::InputPort<double>("current_z"),
        BT::BidirectionalPort<size_t>("current_waypoint"),
        BT::BidirectionalPort<double>("prev_norm_dist"),
    };
  }

  /**
   * @brief Logs the mission size and delegates to onRunning.
   * @return The status of the first onRunning tick.
   */
  BT::NodeStatus onStart() override {
    auto wps = getInput<std::vector<utils::Waypoint>>("active_waypoints").value();
    auto idx = getInput<size_t>("current_waypoint").value();
    if (!wps.empty() && idx < wps.size()) {
      RCLCPP_INFO(node_->get_logger(), "FollowWaypoints: navigating %zu waypoint(s).", wps.size());
    }
    return onRunning();
  }

  /**
   * @brief Publishes a setpoint toward the current waypoint, advancing on capture or slip.
   * @return RUNNING while navigating, SUCCESS once past the final waypoint.
   */
  BT::NodeStatus onRunning() override {
    auto wps = getInput<std::vector<utils::Waypoint>>("active_waypoints").value();
    auto idx = getInput<size_t>("current_waypoint").value();

    if (wps.empty() || idx >= wps.size()) {
      publishStop();
      if (!wps.empty()) {
        RCLCPP_INFO(node_->get_logger(), "FollowWaypoints: completed waypoint navigation.");
      }
      return BT::NodeStatus::SUCCESS;
    }

    double cx = getInput<double>("current_x").value();
    double cy = getInput<double>("current_y").value();
    double cz = getInput<double>("current_z").value();
    double prev_dist = getInput<double>("prev_norm_dist").value();

    const auto& target = wps[idx];
    publishHSD(target, cx, cy);
    double h_dist = std::hypot(target.position.x - cx, target.position.y - cy);
    double v_dist = (target.mode == coug_interfaces::msg::ControlSetpoint::ALTITUDE)
                        ? 0.0
                        : std::abs(target.position.z - cz);

    double norm_cap = std::hypot(h_dist / target.capture_radius, v_dist / target.capture_radius_z);
    double norm_slip = std::hypot(h_dist / target.slip_radius, v_dist / target.slip_radius_z);

    bool capture = norm_cap < 1.0;
    bool slip = (prev_dist > 0.0 && norm_cap > prev_dist && norm_slip < 1.0);

    if (capture || slip) {
      RCLCPP_INFO(node_->get_logger(), "FollowWaypoints: reached waypoint %zu/%zu (%s).", idx + 1,
                  wps.size(), capture ? "capture" : "slip");
      setOutput("current_waypoint", idx + 1);
      setOutput("prev_norm_dist", -1.0);  // new target, reset slip baseline
    } else {
      setOutput("prev_norm_dist", norm_cap);
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { publishStop(); }

 private:
  /**
   * @brief Publishes a heading/speed/depth setpoint from the current position toward the target.
   * @param target The waypoint to steer toward.
   * @param cx The current ENU x position.
   * @param cy The current ENU y position.
   */
  void publishHSD(const utils::Waypoint& target, double cx, double cy) {
    double dx = target.position.x - cx;
    double dy = target.position.y - cy;

    coug_interfaces::msg::ControlSetpoint msg;
    msg.heading = std::atan2(dy, dx) * 180.0 / M_PI;
    msg.speed = target.speed;
    msg.depth = target.position.z;
    msg.mode = target.mode;
    hsd_pub_->publish(msg);
  }

  /**
   * @brief Publishes a zero setpoint to stop the AUV.
   */
  void publishStop() {
    coug_interfaces::msg::ControlSetpoint msg;
    hsd_pub_->publish(msg);
  }

  // --- ROS Interfaces ---
  rclcpp::Publisher<coug_interfaces::msg::ControlSetpoint>::SharedPtr hsd_pub_;
};

}  // namespace coug_helm::bt_nodes
