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

#include "coug_helm/bt_helm.hpp"

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/decorators/loop_node.h>
#include <behaviortree_cpp/json_export.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_cpp/utils/shared_library.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <exception>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <map>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // NOLINT(misc-include-cleaner)
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vector>

#include "coug_helm/bt_helm_parameters.hpp"
#include "coug_helm/bt_nodes/abort_on_teleop.hpp"
#include "coug_helm/bt_nodes/back_up.hpp"
#include "coug_helm/bt_nodes/compute_astronaut_pose.hpp"
#include "coug_helm/bt_nodes/compute_home_waypoint.hpp"
#include "coug_helm/bt_nodes/compute_surface_waypoint.hpp"
#include "coug_helm/bt_nodes/compute_tag_pose.hpp"
#include "coug_helm/bt_nodes/disarm_thruster.hpp"
#include "coug_helm/bt_nodes/emergency_surface.hpp"
#include "coug_helm/bt_nodes/flash_leds.hpp"
#include "coug_helm/bt_nodes/is_astronaut_visible.hpp"
#include "coug_helm/bt_nodes/is_odom_healthy.hpp"
#include "coug_helm/bt_nodes/is_tag_detected.hpp"
#include "coug_helm/bt_nodes/is_waypoints_received.hpp"
#include "coug_helm/bt_nodes/load_behavior.hpp"
#include "coug_helm/bt_nodes/load_command.hpp"
#include "coug_helm/bt_nodes/load_goal.hpp"
#include "coug_helm/bt_nodes/load_search_poses.hpp"
#include "coug_helm/bt_nodes/load_tag_id.hpp"
#include "coug_helm/bt_nodes/load_waypoints.hpp"
#include "coug_helm/bt_nodes/loop_waypoints.hpp"
#include "coug_helm/bt_nodes/navigate_to_pose.hpp"
#include "coug_helm/bt_nodes/navigate_to_tracked_pose.hpp"
#include "coug_helm/bt_nodes/navigate_to_waypoint.hpp"
#include "coug_helm/bt_nodes/pick_up_tool.hpp"
#include "coug_helm/bt_nodes/place_tool.hpp"
#include "coug_helm/bt_nodes/progress_checker.hpp"
#include "coug_helm/bt_nodes/report_command_outcome.hpp"
#include "coug_helm/bt_nodes/report_outcome.hpp"
#include "coug_helm/bt_nodes/reset_localization.hpp"
#include "coug_helm/bt_nodes/stop.hpp"
#include "coug_helm/utils/behavior_enums.hpp"
#include "coug_helm/utils/json_converters.hpp"  // NOLINT(misc-include-cleaner)
#include "coug_interfaces/msg/dvl_beam_list.hpp"
#include "coug_interfaces/msg/way_point.hpp"
#include "coug_interfaces/msg/way_point_list.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace coug_helm {

using aruco_opencv_msgs::msg::ArucoDetection;
using coug_interfaces::msg::DvlBeamList;
using coug_interfaces::msg::WayPoint;
using coug_interfaces::msg::WayPointList;
using geometry_msgs::msg::TwistStamped;
using std_msgs::msg::ColorRGBA;
using utils::AssistCommand;
using utils::Behavior;
using utils::toString;

namespace {

struct Rgb {
  float r;
  float g;
  float b;
};

constexpr Rgb kLedOff{160.0F / 255.0F, 160.0F / 255.0F, 164.0F / 255.0F};
constexpr Rgb kLedRed{1.0F, 0.0F, 0.0F};
constexpr Rgb kLedBlue{85.0F / 255.0F, 170.0F / 255.0F, 1.0F};
constexpr Rgb kLedGreen{0.0F, 1.0F, 0.0F};

auto makeColor(const Rgb& rgb) -> ColorRGBA {
  ColorRGBA color;
  color.r = rgb.r;
  color.g = rgb.g;
  color.b = rgb.b;
  color.a = 1.0F;
  return color;
}

}  // namespace

BtHelmNode::BtHelmNode(const rclcpp::NodeOptions& options)
    : Node("bt_helm_node", options), diagnostic_updater_(this) {
  param_listener_ = std::make_shared<bt_helm_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- Blackboard ---
  blackboard_ = BT::Blackboard::create();

  blackboard_->set("node", std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}));
  blackboard_->set("hsd_topic", params_.hsd_topic);
  blackboard_->set("arm_thruster_service", params_.arm_thruster_service);
  blackboard_->set("reset_localization_service", params_.reset_localization_service);

  const auto server_timeout =
      std::chrono::milliseconds(static_cast<int64_t>(params_.server_timeout_sec * 1000.0));
  blackboard_->set("bt_loop_duration",
                   std::chrono::milliseconds(static_cast<int>(1000.0 / params_.tick_rate_hz)));
  blackboard_->set("server_timeout", server_timeout);
  blackboard_->set("cancel_timeout", server_timeout);
  blackboard_->set(
      "wait_for_service_timeout",
      std::chrono::milliseconds(static_cast<int64_t>(params_.startup_timeout_sec * 1000.0)));

  blackboard_->set("pending_behavior", static_cast<int>(Behavior::kStop));
  blackboard_->set("active_behavior", static_cast<int>(Behavior::kStop));
  blackboard_->set("pending_command", static_cast<int>(AssistCommand::kStay));
  blackboard_->set("active_command", static_cast<int>(AssistCommand::kStay));
  blackboard_->set("flash_start_time", -1.0);
  blackboard_->set("last_teleop_time", -1.0);

  blackboard_->set("waypoint_idx", size_t{0});
  blackboard_->set("active_waypoints", std::vector<WayPoint>{});
  blackboard_->set("mission_waypoints", std::vector<WayPoint>{});
  blackboard_->set("detected_tags", std::map<int, geometry_msgs::msg::Point>{});

  blackboard_->set("curr_x", 0.0);
  blackboard_->set("curr_y", 0.0);
  blackboard_->set("curr_z", 0.0);
  blackboard_->set("curr_heading_degrees", 0.0);
  blackboard_->set("curr_altitude", 0.0);
  blackboard_->set("has_odom", false);
  blackboard_->set("last_odom_time", 0.0);
  blackboard_->set("map_frame", params_.map_frame);

  blackboard_->set("surface_capture_radius", params_.surface_capture_radius);
  blackboard_->set("surface_capture_radius_z", params_.surface_capture_radius_z);
  blackboard_->set("home_capture_radius", params_.home_capture_radius);
  blackboard_->set("home_capture_radius_z", params_.home_capture_radius_z);
  blackboard_->set("surface_slip_radius", params_.surface_slip_radius);
  blackboard_->set("surface_slip_radius_z", params_.surface_slip_radius_z);
  blackboard_->set("home_slip_radius", params_.home_slip_radius);
  blackboard_->set("home_slip_radius_z", params_.home_slip_radius_z);

  blackboard_->set("default_speed_rpm", params_.default_speed_rpm);

  blackboard_->set("odom_timeout_sec", params_.odom_timeout_sec);
  blackboard_->set("odom_recovery_timeout_msec",
                   static_cast<unsigned>(params_.odom_recovery_timeout_sec * 1000.0));

  blackboard_->set("progress_timeout_sec", params_.progress_timeout_sec);
  blackboard_->set("progress_threshold", params_.progress_threshold);
  blackboard_->set("number_of_retries", static_cast<int>(params_.number_of_retries));
  blackboard_->set("wait_duration_msec", static_cast<unsigned>(params_.wait_duration_sec * 1000.0));
  blackboard_->set("backup_speed_rpm", params_.backup_speed_rpm);
  blackboard_->set("backup_duration_sec", params_.backup_duration_sec);

  blackboard_->set("led_flash_duration_msec",
                   static_cast<unsigned>(params_.led_flash_duration_sec * 1000.0));

  // --- ROS Interfaces ---
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  waypoint_sub_ = create_subscription<WayPointList>(
      params_.waypoint_topic, rclcpp::SystemDefaultsQoS(),
      [this](const WayPointList::ConstSharedPtr& msg) { waypointCallback(msg); });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { odomCallback(msg); });

  beams_sub_ = create_subscription<DvlBeamList>(
      params_.beams_topic, rclcpp::SystemDefaultsQoS(),
      [this](const DvlBeamList::ConstSharedPtr& msg) { beamsCallback(msg); });

  aruco_sub_ = create_subscription<ArucoDetection>(
      params_.aruco_topic, rclcpp::SystemDefaultsQoS(),
      [this](const ArucoDetection::ConstSharedPtr& msg) { arucoCallback(msg); });

  teleop_sub_ = create_subscription<TwistStamped>(
      params_.teleop_topic, rclcpp::SystemDefaultsQoS(),
      [this](const TwistStamped::ConstSharedPtr& msg) { teleopCallback(msg); });

  led_color_pub_ =
      create_publisher<ColorRGBA>(params_.led_color_topic, rclcpp::SystemDefaultsQoS());

  start_srv_ = createBehaviorService(params_.start_service, Behavior::kMission);
  stop_srv_ = createBehaviorService(params_.stop_service, Behavior::kStop);
  surface_srv_ = createBehaviorService(params_.surface_service, Behavior::kSurface);
  home_srv_ = createBehaviorService(params_.home_service, Behavior::kHome);
  emergency_stop_srv_ =
      createBehaviorService(params_.emergency_stop_service, Behavior::kEmergencyStop);
  emergency_surface_srv_ =
      createBehaviorService(params_.emergency_surface_service, Behavior::kEmergencySurface);
  assist_srv_ = createBehaviorService(params_.assist_service, Behavior::kAssist);
  follow_srv_ = createAssistCommandService(params_.follow_service, AssistCommand::kFollow);
  stay_srv_ = createAssistCommandService(params_.stay_service, AssistCommand::kStay);
  fetch_srv_ = createAssistCommandService(params_.fetch_service, AssistCommand::kFetch);
  come_srv_ = createAssistCommandService(params_.come_service, AssistCommand::kCome);
  give_srv_ = createAssistCommandService(params_.give_service, AssistCommand::kGive);

  // --- Behavior Tree ---
  factory_.registerNodeType<bt_nodes::IsAstronautVisible>("IsAstronautVisible");
  factory_.registerNodeType<bt_nodes::IsOdomHealthy>("IsOdomHealthy");
  factory_.registerNodeType<bt_nodes::IsTagDetected>("IsTagDetected");
  factory_.registerNodeType<bt_nodes::IsWaypointsReceived>("IsWaypointsReceived");
  factory_.registerNodeType<bt_nodes::AbortOnTeleop>("AbortOnTeleop");
  factory_.registerNodeType<bt_nodes::BackUp>("BackUp");
  factory_.registerNodeType<bt_nodes::ComputeAstronautPose>("ComputeAstronautPose");
  factory_.registerNodeType<bt_nodes::ComputeHomeWaypoint>("ComputeHomeWaypoint");
  factory_.registerNodeType<bt_nodes::ComputeSurfaceWaypoint>("ComputeSurfaceWaypoint");
  factory_.registerNodeType<bt_nodes::ComputeTagPose>("ComputeTagPose");
  factory_.registerNodeType<bt_nodes::DisarmThruster>("DisarmThruster");
  factory_.registerNodeType<bt_nodes::EmergencySurface>("EmergencySurface");
  factory_.registerNodeType<bt_nodes::FlashLeds>("FlashLeds");
  factory_.registerNodeType<bt_nodes::LoadBehavior>("LoadBehavior");
  factory_.registerNodeType<bt_nodes::LoadCommand>("LoadCommand");
  factory_.registerNodeType<bt_nodes::LoadGoal>("LoadGoal");
  factory_.registerNodeType<bt_nodes::LoadSearchPoses>("LoadSearchPoses");
  factory_.registerNodeType<bt_nodes::LoadTagId>("LoadTagId");
  factory_.registerNodeType<bt_nodes::LoadWaypoints>("LoadWaypoints");
  factory_.registerNodeType<bt_nodes::NavigateToWaypoint>("NavigateToWaypoint");
  factory_.registerNodeType<bt_nodes::PickUpTool>("PickUpTool");
  factory_.registerNodeType<bt_nodes::PlaceTool>("PlaceTool");
  factory_.registerNodeType<bt_nodes::ResetLocalization>("ResetLocalization");
  factory_.registerNodeType<bt_nodes::Stop>("Stop");
  factory_.registerNodeType<bt_nodes::LoopWaypoints>("LoopWaypoints");
  factory_.registerNodeType<bt_nodes::ProgressChecker>("ProgressChecker");
  factory_.registerNodeType<bt_nodes::ReportOutcome>("ReportOutcome");
  factory_.registerNodeType<bt_nodes::ReportCommandOutcome>("ReportCommandOutcome");
  factory_.registerNodeType<BT::LoopNode<geometry_msgs::msg::PoseStamped>>("LoopPose");

  factory_.registerBuilder<bt_nodes::NavigateToPose>(
      "NavigateToPose", [](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<bt_nodes::NavigateToPose>(name, "navigate_to_pose", config);
      });
  factory_.registerBuilder<bt_nodes::NavigateToTrackedPose>(
      "NavigateToTrackedPose", [](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<bt_nodes::NavigateToTrackedPose>(name, "navigate_to_pose", config);
      });

  for (const auto& plugin : params_.plugin_lib_names) {
    try {
      factory_.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to register plugin '%s': %s", plugin.c_str(), e.what());
    }
  }

  factory_.registerScriptingEnums<Behavior>();
  factory_.registerScriptingEnums<AssistCommand>();
  factory_.registerScriptingEnum("kGps", WayPoint::GPS);
  factory_.registerScriptingEnum("kAruco", WayPoint::ARUCO);
  BT::RegisterJsonDefinition<WayPoint>();
  BT::RegisterJsonDefinition<std::vector<WayPoint>>();

  const std::string pkg_share = ament_index_cpp::get_package_share_directory("coug_helm");
  const std::string tree_file = params_.tree_file.empty()
                                    ? pkg_share + "/trees/behaviors_hsd_w_recovery.xml"
                                    : params_.tree_file;
  RCLCPP_INFO(get_logger(), "Loading behavior tree: '%s'.", tree_file.c_str());
  tree_ = factory_.createTreeFromFile(tree_file, blackboard_);

  if (params_.publish_groot2) {
    groot2_pub_ = std::make_unique<BT::Groot2Publisher>(tree_, params_.groot2_port);
    RCLCPP_INFO(get_logger(), "Groot2 publisher started on port %ld.", params_.groot2_port);
  }

  tick_timer_ = create_timer(std::chrono::duration<double>(1.0 / params_.tick_rate_hz), [this] {
    tree_.tickOnce();
    publishStatusLed();
  });

  // --- Diagnostics ---
  if (params_.publish_diagnostics) {
    const std::string ns = this->get_namespace();
    const std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/bt_helm_node");

    const std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";

    const std::string behavior_task = prefix + "Behavior Status";
    diagnostic_updater_.add(behavior_task, this, &BtHelmNode::checkBehaviorStatus);
  }

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void BtHelmNode::waypointCallback(const WayPointList::ConstSharedPtr& msg) {
  if (msg->waypoints.empty()) {
    blackboard_->set("mission_waypoints", std::vector<WayPoint>{});
    RCLCPP_INFO(get_logger(), "Mission cleared.");
    return;
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != params_.map_frame) {
    RCLCPP_ERROR(get_logger(), "Mission rejected: waypoints are in '%s', expected '%s'.",
                 msg->header.frame_id.c_str(), params_.map_frame.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Mission received: %zu waypoint(s) in '%s'.", msg->waypoints.size(),
              msg->header.frame_id.c_str());
  for (size_t i = 0; i < msg->waypoints.size(); ++i) {
    const auto& waypoint = msg->waypoints[i];
    RCLCPP_INFO(get_logger(),
                "Waypoint %zu: position (%.2f, %.2f, %.2f) m, speed %.0f RPM, "
                "capture %.1f/%.1f m, slip %.1f/%.1f m (horizontal/vertical).",
                i + 1, waypoint.position.x, waypoint.position.y, waypoint.position.z,
                waypoint.speed_rpm, waypoint.capture_radius, waypoint.capture_radius_z,
                waypoint.slip_radius, waypoint.slip_radius_z);
  }

  blackboard_->set("mission_waypoints", msg->waypoints);
}

void BtHelmNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  blackboard_->set("has_odom", true);
  blackboard_->set("last_odom_time", now().seconds());
  blackboard_->set("curr_x", msg->pose.pose.position.x);
  blackboard_->set("curr_y", msg->pose.pose.position.y);
  blackboard_->set("curr_z", msg->pose.pose.position.z);

  static constexpr double kRadToDeg = 180.0 / M_PI;
  blackboard_->set("curr_heading_degrees", tf2::getYaw(msg->pose.pose.orientation) * kRadToDeg);
}

void BtHelmNode::beamsCallback(const DvlBeamList::ConstSharedPtr& msg) {
  if (!msg->altitude_valid) {
    return;
  }
  blackboard_->set("curr_altitude", msg->altitude);
}

void BtHelmNode::arucoCallback(const ArucoDetection::ConstSharedPtr& msg) {
  if (msg->markers.empty()) {
    return;
  }

  const std::string camera_frame = msg->header.frame_id;

  geometry_msgs::msg::TransformStamped map_T_camera_tf;
  try {
    map_T_camera_tf =
        tf_buffer_->lookupTransform(params_.map_frame, camera_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Failed to look up transform from '%s' to '%s': %s", camera_frame.c_str(),
                         params_.map_frame.c_str(), ex.what());
    return;
  }

  tf2::Transform map_T_camera;
  tf2::fromMsg(map_T_camera_tf.transform, map_T_camera);

  auto tags = blackboard_->get<std::map<int, geometry_msgs::msg::Point>>("detected_tags");
  for (const auto& marker : msg->markers) {
    tf2::Vector3 camera_p_tag;
    tf2::fromMsg(marker.pose.position, camera_p_tag);
    const tf2::Vector3 map_p_tag = map_T_camera * camera_p_tag;

    if (tags.find(marker.marker_id) == tags.end()) {
      RCLCPP_INFO(get_logger(), "Tag %d detected at (%.1f, %.1f) m.", marker.marker_id,
                  map_p_tag.x(), map_p_tag.y());
    }
    tf2::toMsg(map_p_tag, tags[marker.marker_id]);
  }
  blackboard_->set("detected_tags", tags);
}

void BtHelmNode::teleopCallback(const TwistStamped::ConstSharedPtr& msg) {
  if (msg->twist == geometry_msgs::msg::Twist{}) {
    return;
  }
  blackboard_->set("last_teleop_time", now().seconds());
  if (!teleop_active_) {
    teleop_active_ = true;
    RCLCPP_INFO(get_logger(), "Teleop active.");
  }
}

auto BtHelmNode::createBehaviorService(const std::string& service, Behavior behavior)
    -> rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr {
  return create_service<std_srvs::srv::Trigger>(
      service, [this, behavior](const std_srvs::srv::Trigger::Request::SharedPtr&,
                                const std_srvs::srv::Trigger::Response::SharedPtr& res) {
        const auto active = activeBehavior();
        tree_.haltTree();
        if (behavior == Behavior::kMission) {
          blackboard_->set("detected_tags", std::map<int, geometry_msgs::msg::Point>{});
        }
        blackboard_->set("pending_behavior", static_cast<int>(behavior));
        res->success = true;
        res->message = active == behavior ? "Restarting " + toString(behavior) + "."
                                          : "Switching from " + toString(active) + " to " +
                                                toString(behavior) + ".";
        if (active == behavior) {
          RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
        }
      });
}

auto BtHelmNode::createAssistCommandService(const std::string& service, AssistCommand command)
    -> rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr {
  return create_service<std_srvs::srv::Trigger>(
      service, [this, command](const std_srvs::srv::Trigger::Request::SharedPtr&,
                               const std_srvs::srv::Trigger::Response::SharedPtr& res) {
        const auto active = activeBehavior();
        if (active != Behavior::kAssist) {
          res->success = false;
          res->message = "Ignoring " + toString(command) + ": ASSIST not active.";
          RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
          return;
        }
        blackboard_->set("pending_command", static_cast<int>(command));
        res->success = true;
        res->message = "Assist command: " + toString(command) + ".";
      });
}

auto BtHelmNode::activeBehavior() const -> Behavior {
  return static_cast<Behavior>(blackboard_->get<int>("active_behavior"));
}

void BtHelmNode::publishStatusLed() {
  const double now_sec = now().seconds();
  const auto flash_start = blackboard_->get<double>("flash_start_time");
  const auto active = activeBehavior();

  if (teleop_active_ &&
      now_sec - blackboard_->get<double>("last_teleop_time") >= params_.teleop_timeout_sec) {
    teleop_active_ = false;
    RCLCPP_INFO(get_logger(), "Teleop inactive (no input for %.1f s).", params_.teleop_timeout_sec);
  }

  Rgb color = kLedOff;
  if (flash_start >= 0.0 && now_sec - flash_start < params_.led_flash_duration_sec) {
    const bool flash_on =
        static_cast<int>((now_sec - flash_start) * params_.led_flash_rate_hz * 2.0) % 2 == 0;
    color = flash_on ? kLedGreen : kLedOff;
  } else if (teleop_active_) {
    color = kLedBlue;
  } else if (utils::isAutonomous(active)) {
    color = kLedRed;
  }
  led_color_pub_->publish(makeColor(color));
}

void BtHelmNode::checkBehaviorStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  const auto active = activeBehavior();
  stat.summary(utils::isEmergency(active) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                          : diagnostic_msgs::msg::DiagnosticStatus::OK,
               toString(active));

  const auto waypoints = blackboard_->get<std::vector<WayPoint>>("active_waypoints");
  if (!utils::isNavigating(active) || waypoints.empty()) {
    return;
  }

  const auto waypoint_idx = blackboard_->get<size_t>("waypoint_idx");
  if (waypoint_idx >= waypoints.size()) {
    return;
  }

  const auto curr_x = blackboard_->get<double>("curr_x");
  const auto curr_y = blackboard_->get<double>("curr_y");
  const auto& target = waypoints[waypoint_idx];
  const bool altitude_mode = (target.mode == WayPoint::ALTITUDE);
  const auto curr_vertical = blackboard_->get<double>(altitude_mode ? "curr_altitude" : "curr_z");
  // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg)
  stat.addf("Waypoint", "%zu/%zu", waypoint_idx + 1, waypoints.size());
  stat.addf("Horizontal Distance (m)", "%.1f",
            std::hypot(target.position.x - curr_x, target.position.y - curr_y));
  stat.addf(altitude_mode ? "Altitude Error (m)" : "Depth Error (m)", "%.1f",
            std::abs(target.position.z - curr_vertical));
  // NOLINTEND(cppcoreguidelines-pro-type-vararg)
}

}  // namespace coug_helm

RCLCPP_COMPONENTS_REGISTER_NODE(coug_helm::BtHelmNode)
