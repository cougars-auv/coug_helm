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

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class DisarmThruster : public RosBtNode<BT::StatefulActionNode> {
 public:
  DisarmThruster(std::string const& name, BT::NodeConfig const& config)
      : RosBtNode<BT::StatefulActionNode>(name, config),
        service_name_(config.blackboard->get<std::string>("arm_thruster_service")) {
    client_ = node_->create_client<std_srvs::srv::SetBool>(service_name_);
  }

  static auto providedPorts() -> BT::PortsList { return {}; }

  auto onStart() -> BT::NodeStatus override {
    if (!client_->service_is_ready()) {
      RCLCPP_ERROR(node_->get_logger(), "DisarmThruster: service '%s' unavailable.",
                   service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    future_ = client_->async_send_request(request).future;
    return BT::NodeStatus::RUNNING;
  }

  auto onRunning() -> BT::NodeStatus override {
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    bool success = false;
    try {
      success = future_.get()->success;
    } catch (std::exception const& e) {
      RCLCPP_ERROR(node_->get_logger(), "DisarmThruster: %s", e.what());
    }
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "DisarmThruster: succeeded.");
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(node_->get_logger(), "DisarmThruster: failed.");
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override {}

 private:
  std::string service_name_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future_;
};

}  // namespace coug_helm::bt_nodes
