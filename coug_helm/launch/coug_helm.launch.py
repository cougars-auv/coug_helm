# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def agent_frame(agent_ns: SomeSubstitutionsType, frame: str) -> PythonExpression:
    return PythonExpression(["'", agent_ns, f"/{frame}' if '", agent_ns, f"' != '' else '{frame}'"])


def load_launch_params(path: str, top_key: str) -> dict[str, Any]:
    try:
        with open(path) as config_file:
            config = yaml.safe_load(config_file)
        params = config[top_key]["coug_helm_launch"]["ros__parameters"]
        return dict(params)
    except (KeyError, TypeError, OSError):
        return {}


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")

    agent_ns_str = agent_ns.perform(context)

    config_dir = os.environ["CONFIG_DIR"]
    coug_helm_dir = get_package_share_directory("coug_helm")

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_helm_params.yaml"]
    )
    agent_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), [agent_ns, "_params.yaml"]]
    )
    scenario_param_file = (
        LaunchConfiguration("scenario_param_file").perform(context) or agent_param_file
    )

    fleet_launch_params = load_launch_params(
        os.path.join(config_dir, "fleet", "coug_helm_params.yaml"), "/**"
    )
    agent_launch_params = load_launch_params(
        os.path.join(config_dir, f"{agent_ns_str}_params.yaml"), f"/{agent_ns_str}"
    )
    tree_filename = agent_launch_params.get("tree_file", fleet_launch_params.get("tree_file"))
    tree_file = os.path.join(coug_helm_dir, "trees", tree_filename)

    with open(tree_file) as tree:
        use_docking = str("<DockRobot" in tree.read())

    return [
        Node(
            package="coug_helm",
            executable="bt_helm",
            name="bt_helm_node",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "tree_file": tree_file,
                },
            ],
        ),
        Node(
            package="opennav_docking",
            executable="opennav_docking",
            name="docking_server",
            condition=IfCondition(use_docking),
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "fixed_frame": agent_frame(agent_ns, "odom"),
                    "base_frame": agent_frame(agent_ns, "base_link"),
                },
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_docking",
            condition=IfCondition(use_docking),
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "node_names": ["docking_server"],
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "agent_ns",
                default_value="auv0",
            ),
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value="",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
