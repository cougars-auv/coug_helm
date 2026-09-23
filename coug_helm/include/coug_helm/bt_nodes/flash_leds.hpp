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
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "coug_helm/bt_nodes/service_bt_node.hpp"

namespace coug_helm::bt_nodes {

class FlashLeds : public ServiceBtNode<std_srvs::srv::Trigger> {
 public:
  FlashLeds(const std::string& name, const BT::NodeConfig& config)
      : ServiceBtNode(name, config, "flash_leds_service") {}

  static auto providedPorts() -> BT::PortsList { return {}; }

 protected:
  [[nodiscard]] auto makeRequest() const -> std_srvs::srv::Trigger::Request::SharedPtr override {
    return std::make_shared<std_srvs::srv::Trigger::Request>();
  }
};

}  // namespace coug_helm::bt_nodes
