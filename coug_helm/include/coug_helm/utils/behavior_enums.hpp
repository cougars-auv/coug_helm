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
 * @file behavior_enums.hpp
 * @brief Behavior modes resolved by the tree and shared with diagnostics.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <string>

namespace coug_helm::utils {

/**
 * @enum Behavior
 * @brief Mission mode the tree dispatches on (registered as a BT scripting enum).
 */
enum class Behavior : int {
  STOP = 0,
  MISSION,
  SURFACE,
  HOME,
  EMERGENCY_STOP,
  EMERGENCY_SURFACE,
};

/**
 * @brief Maps a Behavior to its name for logging and diagnostics.
 * @param behavior The Behavior enum value to convert.
 * @return The string representation of the Behavior.
 */
inline std::string toString(Behavior behavior) {
  switch (behavior) {
    case Behavior::STOP:
      return "STOP";
    case Behavior::MISSION:
      return "MISSION";
    case Behavior::SURFACE:
      return "SURFACE";
    case Behavior::HOME:
      return "HOME";
    case Behavior::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case Behavior::EMERGENCY_SURFACE:
      return "EMERGENCY_SURFACE";
  }
  return "UNKNOWN";
}

}  // namespace coug_helm::utils
