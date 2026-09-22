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

#include <chrono>
#include <exception>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

template <class ServiceT>
class ServiceBtNode : public RosBtNode<BT::StatefulActionNode> {
 public:
  ServiceBtNode(const std::string& name, const BT::NodeConfig& config,
                const std::string& service_key)
      : RosBtNode<BT::StatefulActionNode>(name, config),
        service_name_(config.blackboard->get<std::string>(service_key)),
        client_(node_->create_client<ServiceT>(service_name_)) {}

  auto onStart() -> BT::NodeStatus override {
    if (!client_->service_is_ready()) {
      RCLCPP_ERROR(node_->get_logger(), "%s: service '%s' unavailable.", registrationName().c_str(),
                   service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    future_ = client_->async_send_request(makeRequest()).future;
    return BT::NodeStatus::RUNNING;
  }

  auto onRunning() -> BT::NodeStatus override {
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    bool success = false;
    try {
      success = future_.get()->success;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "%s: request failed (%s).", registrationName().c_str(),
                   e.what());
    }
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "%s: succeeded.", registrationName().c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: failed.", registrationName().c_str());
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override {}

 protected:
  [[nodiscard]] virtual auto makeRequest() const -> typename ServiceT::Request::SharedPtr = 0;

 private:
  std::string service_name_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  typename rclcpp::Client<ServiceT>::SharedFuture future_;
};

}  // namespace coug_helm::bt_nodes
