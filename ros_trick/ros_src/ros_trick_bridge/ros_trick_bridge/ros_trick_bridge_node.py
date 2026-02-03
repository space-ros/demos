# Copyright 2024 Blazej Fiderek (xfiderek)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import subprocess
import threading
import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from ros_trick_bridge.trick_variable_server_client import TrickVariableServerClient
from rosgraph_msgs.msg import Clock


class RosTrickBridgeNode(Node):
    TRICK_TICK_VALUE_VAR_NAME = "trick_sys.sched.time_tic_value"
    TRICK_TIME_TICS_VAR_NAME = "trick_sys.sched.time_tics"

    def __init__(self, node_name: str, **kwargs):
        self._trick_subprocess = None
        self._std_print_period = 0.1
        self._stdout_print_thread = None

        self._trick_variable_server_client = None
        self._clock_publisher = None

        super().__init__(node_name, **kwargs)

    # def on_init(self, state: State)  -> TransitionCallbackReturn:
    #     self.get_logger().info('nddd')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Declare ROS 2 parameters
        self.declare_parameter("sim_directory", "/path/to/sim")
        self.declare_parameter("sim_executable", "S_main_Linux_11.4_x86_64.exe")
        self.declare_parameter("sim_inputfile", "RUN_2DPlanar/input.py")
        self.declare_parameter("sim_args", "")
        self.declare_parameter("trick_host", "localhost")
        self.declare_parameter("trick_port", 49765)
        self.declare_parameter("publish_clock", True)
        self.declare_parameter("clock_publish_period", 0.1)

        self.get_logger().info(f"Configuring from state: {state.label}")
        if not "TRICK_HOME" in os.environ:
            error = "TRICK_HOME env variable is not set, terminating"
            self.get_logger().error(error)
            raise Exception(error)

        # Get the parameters
        sim_directory = (
            self.get_parameter("sim_directory").get_parameter_value().string_value + "/"
        )
        sim_executable = (
            self.get_parameter("sim_executable").get_parameter_value().string_value
        )
        sim_inputfile = (
            self.get_parameter("sim_inputfile").get_parameter_value().string_value
        )
        sim_args = self.get_parameter("sim_args").get_parameter_value().string_value
        try:
            cmd = [f"./{sim_executable}", sim_inputfile, sim_args]
            self.get_logger().info(f"Starting trick with command: {cmd}")
            self._trick_subprocess = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                cwd=sim_directory,
            )
            self.get_logger().info("######################################")
            self.get_logger().info("Trick subprocess started successfully.")
            self.get_logger().info("######################################")
            self.get_logger().info(f"Trick pid is {self._trick_subprocess.pid}")
        except Exception as e:
            self.get_logger().error("######################################")
            self.get_logger().error(f"Failed to start trick subprocess: {e}")
            self.get_logger().error("######################################")
            return TransitionCallbackReturn.FAILURE

        # it must run on a separate thread to avoid IO blocking
        self._stdout_print_thread = threading.Thread(
            target=self._print_subprocess_stdout
        )
        self._stdout_print_thread.start()

        # give a process some time to spin up
        time.sleep(1.0)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        if self.get_parameter("publish_clock").get_parameter_value().bool_value:
            trick_data_callback = self._publish_clock_from_trick
            self._clock_publisher = self.create_publisher(
                msg_type=Clock, topic="/clock", qos_profile=10
            )
        else:
            # we don't need any data from trick otherwise
            trick_data_callback = None

        try:
            self._trick_variable_server_client = TrickVariableServerClient(
                self.get_parameter("trick_host").get_parameter_value().string_value,
                self.get_parameter("trick_port").get_parameter_value().integer_value,
                "ros_trick_bridge",
                [self.TRICK_TICK_VALUE_VAR_NAME, self.TRICK_TIME_TICS_VAR_NAME],
                self.get_parameter("clock_publish_period")
                .get_parameter_value()
                .double_value,
                trick_data_callback,
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to start trick variable server client: {e}"
            )
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f"Activating from state: {state.label}")
        self._trick_variable_server_client.unfreeze_sim()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating from state: {state.label}")
        self._trick_variable_server_client.freeze_sim()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up from state: {state.label}")
        self._teardown_sim()

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down from state: {state.label}")
        self._teardown_sim()

        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Error from state: {state.label}")
        return TransitionCallbackReturn.SUCCESS

    def _teardown_sim(self):
        self._trick_variable_server_client = None

        # Terminate the subprocess if it's still running
        if self._trick_subprocess and self._trick_subprocess.poll() is None:
            self._trick_subprocess.terminate()

    def _publish_clock_from_trick(self, trick_data):
        ticks_per_second = int(trick_data[self.TRICK_TICK_VALUE_VAR_NAME])
        curr_time_tics = int(trick_data[self.TRICK_TIME_TICS_VAR_NAME])
        seconds = curr_time_tics // ticks_per_second
        nanoseconds = (curr_time_tics % ticks_per_second) * (
            1000000000 // ticks_per_second
        )
        clock_msg = Clock()
        clock_msg.clock = Time()
        clock_msg.clock.sec = seconds
        clock_msg.clock.nanosec = nanoseconds
        self._clock_publisher.publish(clock_msg)

    def _print_subprocess_stdout(self):
        while True:
            if self._trick_subprocess:
                # Read from stdout
                stdout_line = self._trick_subprocess.stdout.readline()
                if stdout_line:
                    self.get_logger().info(stdout_line)
            time.sleep(self._std_print_period)


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    lifecycle_node = RosTrickBridgeNode("ros_trick_bridge")
    executor.add_node(lifecycle_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lifecycle_node._teardown_sim()


if __name__ == "__main__":
    main()
