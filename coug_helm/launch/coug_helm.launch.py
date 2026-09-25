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
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def is_agent(agent_ns: LaunchConfiguration, *names: str) -> PythonExpression:
    return PythonExpression(["'", agent_ns, "' in ", str(names)])


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
    scenario_param_path = LaunchConfiguration("scenario_param_file").perform(context)

    config_dir = os.environ["CONFIG_DIR"]
    coug_helm_dir = get_package_share_directory("coug_helm")

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_helm_params.yaml"]
    )
    agent_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), [agent_ns, "_params.yaml"]]
    )
    scenario_param_file = scenario_param_path or agent_param_file

    fleet_param_path = os.path.join(config_dir, "fleet", "coug_helm_params.yaml")
    agent_param_path = os.path.join(config_dir, f"{agent_ns_str}_params.yaml")

    launch_params = {
        **load_launch_params(fleet_param_path, "/**"),
        **load_launch_params(agent_param_path, f"/{agent_ns_str}"),
        **load_launch_params(scenario_param_path, "/**"),
        **load_launch_params(scenario_param_path, f"/{agent_ns_str}"),
    }
    tree_filename = launch_params["tree_file"]
    tree_file = os.path.join(coug_helm_dir, "trees", tree_filename)

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
                    "map_frame": "map",
                },
            ],
        ),
        Node(
            package="aruco_opencv",
            executable="aruco_tracker_autostart",
            name="aruco_tracker",
            condition=IfCondition(is_agent(agent_ns, "rover1gz", "rover2gz", "rover3gz")),
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
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
