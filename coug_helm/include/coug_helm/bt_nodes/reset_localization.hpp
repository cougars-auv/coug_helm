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
 * @file reset_localization.hpp
 * @brief BT action node that resets the localization stack after an odometry dropout.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

/**
 * @class ResetLocalization
 * @brief BT action node that resets the localization stack after an odometry dropout.
 */
class ResetLocalization : public RosBtNode<BT::StatefulActionNode> {
 public:
  ResetLocalization(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::StatefulActionNode>(name, config),
        service_name_(config.blackboard->get<std::string>("reset_localization_service")) {
    client_ = node_->create_client<std_srvs::srv::Trigger>(service_name_);
  }

  // --- Overrides ---
  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Dispatch the reset request to the reset localization service.
   * @return FAILURE if the service is unavailable, otherwise RUNNING while awaiting the response.
   */
  BT::NodeStatus onStart() override {
    if (!client_->service_is_ready()) {
      RCLCPP_ERROR(node_->get_logger(), "ResetLocalization: service '%s' unavailable.",
                   service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    future_ =
        client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()).future;
    return BT::NodeStatus::RUNNING;
  }

  /**
   * @brief Poll for the reset response.
   * @return RUNNING until the response arrives, then SUCCESS or FAILURE based on the result.
   */
  BT::NodeStatus onRunning() override {
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    bool success = false;
    try {
      success = future_.get()->success;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "ResetLocalization: %s", e.what());
    }
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "ResetLocalization: succeeded.");
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(node_->get_logger(), "ResetLocalization: failed.");
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override {}

 private:
  std::string service_name_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_;
};

}  // namespace coug_helm::bt_nodes
