// Copyright 2024 Blazej Fiderek (xfiderek)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRICK_ROS2_CONTROL__TRICK_SYSTEM_HPP_
#define TRICK_ROS2_CONTROL__TRICK_SYSTEM_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "trick_ros2_control/socket.hpp"
#include "trick_ros2_control/utils.hpp"
#include "trick_ros2_control/visibility_control.h"
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace trick_ros2_control
{

/**
 * @brief Data structure representing robot data on ROS side
 *
 */
struct RosJointData
{
  std::string name;
  double joint_position = std::numeric_limits<double>::quiet_NaN();
  double joint_velocity = std::numeric_limits<double>::quiet_NaN();
  double joint_acceleration = std::numeric_limits<double>::quiet_NaN();
  double joint_effort = std::numeric_limits<double>::quiet_NaN();
  double joint_position_cmd;
  double joint_velocity_cmd;
  double joint_acceleration_cmd;
  double joint_effort_cmd;
};

/**
 * @brief Data structure representing data on Trick side
 * Used in the file to map trick variable to pointer to the data
 * Used for both sending and receiving data
 */
struct TrickData
{
  const std::string trick_var_name;
  double* const data_ptr;
  TrickData(const std::string& trick_var_name, double* const data_ptr)
    : trick_var_name(trick_var_name), data_ptr(data_ptr){};
};

/**
 * @brief Mapping from the pointer where command is stored to the corresponding trick variable name
 */
using CommandDataToTrickVarName = std::pair<double* const, const std::string>;
/**
 * @brief Mapping from trick variable name to pointer where its value is stored
 */
using TrickVarNameToStateData = std::pair<const std::string, double* const>;

class TrickSystem : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /**
   * @brief Represents data received from trick
   */
  std::vector<TrickData> trick_state_variables_;
  /**
   * @brief Represents data to be sent to trick
   */
  std::vector<TrickData> trick_command_variables_;
  /**
   * @brief Thread used for receiving data from trick asynchronously
   */
  std::thread incoming_data_thread_;
  /**
   * @brief Socket encapsulating the connection with trick simulation
   */
  Socket trick_conn_socket_;
  /**
   * @brief Buffer for accessing incoming data in realtime-safe manner
   */
  realtime_tools::RealtimeBuffer<std::vector<double>> trick_data_incoming_buffer_;

  int trick_server_port_;
  std::string trick_hostname_;
  double trick_variable_cycle_rate_;

  const int MAX_NO_OF_RECONNECTS = 5;
  const int RECONNECT_WAIT_TIME_S = 2;
  std::vector<struct RosJointData> joints_data_;

  // parameter names used in URDF file for trick-related params
  const std::string JOINT_PARAM_STATE_INTERFACE_KEY = "state_interface";
  const std::string JOINT_PARAM_COMMAND_INTERFACE_KEY = "command_interface";
  const std::string JOINT_PARAM_TRICK_VARIABLE_NAME_KEY = "trick_variable_name";
};

}  // namespace trick_ros2_control

#endif  // TRICK_ROS2_CONTROL__TRICK_SYSTEM_HPP_
