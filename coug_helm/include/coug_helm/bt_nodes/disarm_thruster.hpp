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

#include <memory>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#include "coug_helm/bt_nodes/service_bt_node.hpp"

namespace coug_helm::bt_nodes {

class DisarmThruster : public ServiceBtNode<std_srvs::srv::SetBool> {
 public:
  DisarmThruster(const std::string& name, const BT::NodeConfig& config)
      : ServiceBtNode(name, config, "arm_thruster_service") {}

  static auto providedPorts() -> BT::PortsList { return {}; }

 protected:
  auto makeRequest() const -> std_srvs::srv::SetBool::Request::SharedPtr override {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    return request;
  }
};

}  // namespace coug_helm::bt_nodes
