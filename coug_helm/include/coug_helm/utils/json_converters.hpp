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

#include <behaviortree_cpp/json_export.h>

#include <coug_interfaces/msg/way_point.hpp>
#include <nav2_behavior_tree/json_utils.hpp>

namespace coug_interfaces::msg {

BT_JSON_CONVERTER(coug_interfaces::msg::WayPoint, msg) {
  add_field("position", &msg.position);
  add_field("speed_rpm", &msg.speed_rpm);
  add_field("capture_radius", &msg.capture_radius);
  add_field("capture_radius_z", &msg.capture_radius_z);
  add_field("slip_radius", &msg.slip_radius);
  add_field("slip_radius_z", &msg.slip_radius_z);
  add_field("mode", &msg.mode);
  add_field("subwaypoints", &msg.subwaypoints);
  add_field("tag_id", &msg.tag_id);
  add_field("arrival_flash", &msg.arrival_flash);
  add_field("type", &msg.type);
}

}  // namespace coug_interfaces::msg
