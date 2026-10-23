# Copyright 2026 KUKA
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

"""The control loop must never busy-spin, in any state robot_manager can produce."""

import multiprocessing
import os
import sys
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import psutil
import pytest
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import SetHardwareComponentState
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from lifecycle_msgs.msg import State
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from joint_move_test_case import JointMoveTestCase  # noqa: E402

COMPONENT = "kr6_r700_2"

# A sleeping or blocking loop stays far below one core; a spinning loop pegs one.
# Wide margin on both sides so CI noise cannot flip the verdict.
SPIN_THRESHOLD_PERCENT = 50.0
MEASURE_SECONDS = 1.5


def _config(name):
    return os.path.join(get_package_share_directory("kuka_rsi_driver"), "config", name)


def _measure_cpu_percent(pid, seconds=MEASURE_SECONDS):
    # Blocks for `seconds`; percent is relative to a single core (100 == one core busy).
    return psutil.Process(pid).cpu_percent(interval=seconds)


def _spin_forever():
    while True:
        pass


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    get_package_share_directory("kuka_agilus_support"),
                    "urdf",
                    f"{COMPONENT}.urdf.xacro",
                ]
            ),
            " driver_version:=rsi_only client_ip:=0.0.0.0 client_port:=59152 roundtrip_time:=4000",
        ],
        on_stderr="capture",
    )

    control_node = launch_ros.actions.Node(
        package="kuka_drivers_core",
        executable="control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            _config("ros2_controller_config_rsi_only.yaml"),
            {"hardware_components_initial_state": {"unconfigured": [COMPONENT]}},
        ],
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("kuka_rsi_simulator"),
                "/launch/",
                "kuka_rsi_simulator.launch.py",
            ]
        )
    )

    return launch.LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            simulator,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class LoopBlocksOrSleepsEveryCycleInEveryState(JointMoveTestCase):
    def _control_node_pid(self, timeout=20):
        def find_pid():
            for proc in psutil.process_iter(["pid", "cmdline"]):
                cmdline = proc.info["cmdline"] or []
                if any("control_node" in part for part in cmdline):
                    return proc.info["pid"]
            return None

        self._spin_until(lambda: find_pid() is not None, timeout)
        pid = find_pid()
        self.assertIsNotNone(pid, "control_node process not found")
        return pid

    def _cpu_percent(self, pid, seconds=MEASURE_SECONDS):
        return _measure_cpu_percent(pid, seconds)

    def _assert_not_spinning(self, pid, label):
        cpu = self._cpu_percent(pid)
        print(f"control_node CPU in {label}: {cpu:.1f}%")
        self.assertLess(
            cpu,
            SPIN_THRESHOLD_PERCENT,
            f"control_node busy-spins in {label}: {cpu:.1f}% CPU "
            f"(threshold {SPIN_THRESHOLD_PERCENT}%)",
        )

    def _set_hw_state(self, state_id, label):
        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state.id = state_id
        request.target_state.label = label
        result = self._call(
            SetHardwareComponentState,
            "/controller_manager/set_hardware_component_state",
            request,
            timeout=30,
        )
        self.assertTrue(result.ok, f"set_hardware_component_state({label}) refused")

    def test_loop_blocks_or_sleeps_every_cycle_in_every_state(self, proc_output):
        proc_output.assertWaitFor(
            f"Setting component '{COMPONENT}' to 'unconfigured' state.", timeout=30
        )
        pid = self._control_node_pid()

        # Production state: not configured yet, hardware component UNCONFIGURED.
        self._assert_not_spinning(pid, "not configured")

        is_configured = self.node.create_publisher(
            Bool,
            "/robot_manager/is_configured",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.assertTrue(
            self._spin_until(lambda: is_configured.get_subscription_count() > 0, timeout=10),
            "control_node is not subscribed to /robot_manager/is_configured",
        )

        # Known case, not asserted: robot_manager only publishes is_configured=true
        # after configure succeeds, so is_configured=true with an UNCONFIGURED
        # component doesn't happen in production. Log it for visibility only.
        for _ in range(5):
            is_configured.publish(Bool(data=True))
            self._spin_for(0.1)
        cpu = self._cpu_percent(pid)
        print(f"control_node CPU, is_configured=true, UNCONFIGURED (not in prod): {cpu:.1f}%")
        is_configured.publish(Bool(data=False))
        self._spin_for(0.1)

        self._set_hw_state(State.PRIMARY_STATE_INACTIVE, "inactive")
        proc_output.assertWaitFor(f"Successful 'configure' of hardware '{COMPONENT}'", timeout=30)
        for _ in range(5):
            is_configured.publish(Bool(data=True))
            self._spin_for(0.1)
        self._assert_not_spinning(pid, "INACTIVE")

        self._set_hw_state(State.PRIMARY_STATE_ACTIVE, "active")
        proc_output.assertWaitFor(f"Successful 'activate' of hardware '{COMPONENT}'", timeout=30)
        self._assert_not_spinning(pid, "ACTIVE")

        self._set_hw_state(State.PRIMARY_STATE_INACTIVE, "inactive")


class BusySpinDetectorCatchesARealSpin(unittest.TestCase):
    """Negative control: the measurement above must trip on an actual busy-spin."""

    def setUp(self):
        # fork, not the platform default: the test module isn't import-by-name
        # here, and a forkserver/spawn child would need to pickle the target.
        self.proc = multiprocessing.get_context("fork").Process(target=_spin_forever, daemon=True)
        self.proc.start()
        handle = psutil.Process(self.proc.pid)
        handle.cpu_percent(interval=None)  # prime psutil's internal baseline
        try:
            handle.nice(-10)  # improve odds of a fair CPU share on a loaded runner
        except psutil.AccessDenied:
            pass

    def tearDown(self):
        self.proc.terminate()
        self.proc.join(timeout=5)

    def test_spinning_process_trips_the_detector(self):
        cpu = _measure_cpu_percent(self.proc.pid)
        print(f"negative control, deliberately spinning process: {cpu:.1f}%")
        self.assertGreater(
            cpu,
            SPIN_THRESHOLD_PERCENT,
            f"detector failed to catch a deliberate busy-spin: {cpu:.1f}% CPU "
            f"(threshold {SPIN_THRESHOLD_PERCENT}%)",
        )


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
