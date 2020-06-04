#!/usr/bin/env python3
# Copyright 2020 Box Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    this_package = "ouster_tools"
    driver_package = "ros2_ouster"

    log_level = LaunchConfiguration("log_level")

    perf_node_name = "perf_component"
    perf_params = LaunchConfiguration("perf_params")
    perf_share_dir = get_package_share_directory(this_package)

    driver_node_name = "ouster_driver"
    driver_params = LaunchConfiguration("driver_params")
    driver_share_dir = get_package_share_directory(driver_package)

    #---------------------------------------
    # Container args
    #---------------------------------------
    log_level_declare = \
      DeclareLaunchArgument(
          "log_level", default_value="INFO",
          description="ROS logging level"
          )

    #---------------------------------------
    # perf_node
    #---------------------------------------

    perf_params_declare = \
      DeclareLaunchArgument(
          "perf_params",
          default_value=os.path.join(
              perf_share_dir, "etc", "perf_component.yaml"
              ),
          description="Path to perf_node params yaml"
          )

    perf_node = ComposableNode(
        package="ouster_tools",
        node_plugin="ouster_tools::PerfNode",
        node_namespace="/",
        node_name=perf_node_name,
        parameters=[perf_params]
        )

    #---------------------------------------
    # driver_node
    #---------------------------------------

    driver_params_declare = \
      DeclareLaunchArgument(
          "driver_params",
          default_value=os.path.join(
              driver_share_dir, "params", "os1.yaml"
              ),
          description="Path to ouster_driver params yaml"
          )

    driver_node = ComposableNode(
        package="ros2_ouster",
        node_plugin="ros2_ouster::OS1Driver",
        node_namespace="/",
        node_name=driver_node_name,
        parameters=[driver_params]
        )

    #---------------------------------------
    # container
    #---------------------------------------

    container = ComposableNodeContainer(
        node_name="perf_container",
        node_namespace="/",
        package="rclcpp_components",
        node_executable="component_container",
        composable_node_descriptions=[perf_node, driver_node],
        arguments=[("--ros-args"), ("--log-level"), (log_level)],
        output="screen",
        emulate_tty=True,
        log_cmd=True
        )

    #---------------------------------------
    # lifecycle manager for the driver FSM
    #---------------------------------------

    driver_mgr = \
      ExecuteProcess(
          cmd=['ros2', 'launch', this_package,
               'driver_component_manager.launch.py'],
          output="screen",
          log_cmd=True
          )

    return LaunchDescription([
        log_level_declare,
        perf_params_declare,
        driver_params_declare,
        container,
        driver_mgr
        ])
