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
#include <cstdint>
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
        client_(node_->create_client<ServiceT>(service_name_)),
        timeout_(config.blackboard->get<std::chrono::milliseconds>("server_timeout")) {}

  auto onStart() -> BT::NodeStatus override {
    request_sent_ = false;
    start_time_ = std::chrono::steady_clock::now();
    return onRunning();
  }

  auto onRunning() -> BT::NodeStatus override {
    if (!request_sent_) {
      if (!client_->service_is_ready()) {
        if (std::chrono::steady_clock::now() - start_time_ < timeout_) {
          return BT::NodeStatus::RUNNING;
        }
        RCLCPP_ERROR(node_->get_logger(), "%s: service '%s' not available after %ld ms.",
                     registrationName().c_str(), service_name_.c_str(),
                     static_cast<long>(timeout_.count()));
        return BT::NodeStatus::FAILURE;
      }
      auto pending = client_->async_send_request(makeRequest());
      future_ = pending.future.share();
      request_id_ = pending.request_id;
      request_sent_ = true;
      start_time_ = std::chrono::steady_clock::now();
      return BT::NodeStatus::RUNNING;
    }

    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      if (std::chrono::steady_clock::now() - start_time_ < timeout_) {
        return BT::NodeStatus::RUNNING;
      }
      client_->remove_pending_request(request_id_);
      RCLCPP_ERROR(node_->get_logger(), "%s: no response from '%s' within %ld ms.",
                   registrationName().c_str(), service_name_.c_str(),
                   static_cast<long>(timeout_.count()));
      return BT::NodeStatus::FAILURE;
    }
    typename ServiceT::Response::SharedPtr response;
    try {
      response = future_.get();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "%s: request to '%s' failed: %s",
                   registrationName().c_str(), service_name_.c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }
    if (response->success) {
      RCLCPP_INFO(node_->get_logger(), "%s: '%s' succeeded.", registrationName().c_str(),
                  service_name_.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    if (response->message.empty()) {
      RCLCPP_WARN(node_->get_logger(), "%s: '%s' reported failure.", registrationName().c_str(),
                  service_name_.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "%s: '%s' reported failure: %s", registrationName().c_str(),
                  service_name_.c_str(), response->message.c_str());
    }
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override {
    if (request_sent_) {
      client_->remove_pending_request(request_id_);
    }
  }

 protected:
  [[nodiscard]] virtual auto makeRequest() const -> typename ServiceT::Request::SharedPtr = 0;

 private:
  std::string service_name_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  std::chrono::milliseconds timeout_;
  typename rclcpp::Client<ServiceT>::SharedFuture future_;
  int64_t request_id_{};
  bool request_sent_{false};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace coug_helm::bt_nodes
