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

import sys
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

def generate_launch_description():
    package_name = 'ouster_ptp'
    node_namespace = LaunchConfiguration('__ns')
    node_name = LaunchConfiguration('__node')
    node_exe = 'pmc_node'
    share_dir = get_package_share_directory(package_name)
    params_file = LaunchConfiguration('__params')

    ns_declare = \
      DeclareLaunchArgument('__ns', default_value='ouster_ptp',
                            description="Remap the node namespace")
    nn_declare = \
      DeclareLaunchArgument('__node', default_value='pmc_node',
                            description="Remap the node name")

    params_declare = \
      DeclareLaunchArgument(
          '__params', default_value=os.path.join(share_dir, 'etc', 'pmc.yaml'),
          description="Path to the YAML-based node parameterization")

    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = \
      "[{%s}] {%s} [{%s}]: {%s}" % ("severity", "time", "name", "message")

    #------------------------------------------------------------
    # Nodes
    #------------------------------------------------------------

    pmc_node = \
      LifecycleNode(
          package=package_name,
          executable=node_exe,
          namespace=node_namespace,
          name=node_name,
          output='screen',
          log_cmd=True,
          parameters=[params_file],
          emulate_tty=True
          )

    #------------------------------------------------------------
    # Events we need to emit to induce state transitions
    #------------------------------------------------------------

    pmc_configure_evt = \
      EmitEvent(
          event=ChangeState(
              lifecycle_node_matcher = \
                launch.events.matches_action(pmc_node),
              transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
              )
          )

    pmc_activate_evt = \
      EmitEvent(
          event=ChangeState(
              lifecycle_node_matcher = \
                launch.events.matches_action(pmc_node),
              transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
              )
          )

    pmc_cleanup_evt = \
      EmitEvent(
          event=ChangeState(
              lifecycle_node_matcher = \
                launch.events.matches_action(pmc_node),
              transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CLEANUP
              )
          )

    pmc_shutdown_evt = EmitEvent(event=launch.events.Shutdown())

    #------------------------------------------------------------
    # These are the edges of the state machine graph we want to autonomously
    # manage
    #------------------------------------------------------------

    #
    # unconfigured -> configuring -> inactive
    #
    pmc_node_unconfigured_to_inactive_handler = \
      RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node = pmc_node,
              start_state = 'configuring',
              goal_state = 'inactive',
              entities = [
                  LogInfo(msg = "Emitting 'TRANSITION_ACTIVATE' event"),
                  pmc_activate_evt,
                  ],
              )
          )

    #
    # active -> deactivating -> inactive
    #
    pmc_node_active_to_inactive_handler = \
      RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node = pmc_node,
              start_state = 'deactivating',
              goal_state = 'inactive',
              entities = [
                  LogInfo(msg = "Emitting 'TRANSITION_CLEANUP' event"),
                  pmc_cleanup_evt,
                  ],
              )
          )

    #
    # inactive -> cleaningup -> unconfigured
    #
    pmc_node_inactive_to_unconfigured_handler = \
      RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node = pmc_node,
              start_state = 'cleaningup',
              goal_state = 'unconfigured',
              entities = [
                  LogInfo(msg = "Emitting 'TRANSITION_CONFIGURE' event"),
                  pmc_configure_evt,
                  ],
              )
          )

    #
    # * -> errorprocessing -> unconfigured
    #
    pmc_node_errorprocessing_to_unconfigured_handler = \
      RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node = pmc_node,
              start_state = 'errorprocessing',
              goal_state = 'unconfigured',
              entities = [
                  LogInfo(msg = "Emitting 'TRANSITION_CONFIGURE' event"),
                  pmc_configure_evt,
                  ],
              )
          )

    #
    # * -> shuttingdown -> finalized
    #
    pmc_node_shuttingdown_to_finalized_handler = \
      RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node = pmc_node,
              start_state = 'shuttingdown',
              goal_state = 'finalized',
              entities = [
                  LogInfo(msg = "Emitting 'SHUTDOWN' event"),
                  pmc_shutdown_evt,
                  ],
              )
          )

    #------------------------------------------------------------
    # Now, add all the actions to a launch description and return it
    #------------------------------------------------------------

    ld = LaunchDescription()
    ld.add_action(ns_declare)
    ld.add_action(nn_declare)
    ld.add_action(params_declare)
    ld.add_action(pmc_node_unconfigured_to_inactive_handler)
    ld.add_action(pmc_node_active_to_inactive_handler)
    ld.add_action(pmc_node_inactive_to_unconfigured_handler)
    ld.add_action(pmc_node_errorprocessing_to_unconfigured_handler)
    ld.add_action(pmc_node_shuttingdown_to_finalized_handler)
    ld.add_action(pmc_node)
    ld.add_action(pmc_configure_evt)

    return ld
