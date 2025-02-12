// Copyright (c) 2024, Blazej Fiderek (xfiderek), Stogl Robotics Consulting UG
// (haftungsbeschränkt) (template)
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

#include "trick_ros2_control/trick_system.hpp"

namespace trick_ros2_control
{

hardware_interface::CallbackReturn TrickSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("trick_variable_server_host") == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(rclcpp::get_logger("TrickSystem"),
                 "trick_variable_server_host param is not defined. Define in "
                 "it 'ros2_control/hardware' block in your "
                 "robot's URDF");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.hardware_parameters.find("trick_variable_server_port") == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(rclcpp::get_logger("TrickSystem"),
                 "trick_variable_server_port param is not defined.\n Define in "
                 "it 'ros2_control/hardware' block in "
                 "your robot's URDF");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.hardware_parameters.find("trick_variable_cycle_rate") == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(rclcpp::get_logger("TrickSystem"),
                 "trick_variable_cycle_rate param is not defined.\n Define in "
                 "it 'ros2_control/hardware' block in your "
                 "robot's URDF");
    return hardware_interface::CallbackReturn::ERROR;
  }
  trick_hostname_ = info_.hardware_parameters["trick_variable_server_host"];
  trick_server_port_ = std::stoi(info_.hardware_parameters["trick_variable_server_port"]);
  trick_variable_cycle_rate_ = std::stod(info_.hardware_parameters["trick_variable_cycle_rate"]);
  size_t n_joints = info_.joints.size();
  joints_data_.resize(n_joints);
  if (n_joints > 0)
  {
    trick_data_incoming_buffer_.writeFromNonRT(std::vector<double>(n_joints * info.joints[0].state_interfaces.size(),
                                                                   std::numeric_limits<double>::quiet_NaN()));
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrickSystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  bool trick_conn_successful = false;
  for (int i = 0; i < MAX_NO_OF_RECONNECTS && rclcpp::ok(); ++i)
  {
    trick_conn_successful = trick_conn_socket_.init(trick_hostname_, trick_server_port_) >= 0;
    if (trick_conn_successful)
    {
      RCLCPP_INFO(rclcpp::get_logger("TrickSystem"), "Connected to trick server successfully");
      break;
    }
    RCLCPP_INFO(rclcpp::get_logger("TrickSystem"), "Could not connect to trick, retrying %d/%d", i + 1,
                MAX_NO_OF_RECONNECTS);
    std::this_thread::sleep_for(std::chrono::seconds(RECONNECT_WAIT_TIME_S));
  }
  if (!trick_conn_successful)
  {
    std::string err = "could not connect to Trick server on host: " + trick_hostname_ +
                      " and port: " + std::to_string(trick_server_port_);
    RCLCPP_FATAL(rclcpp::get_logger("TrickSystem"), err.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  trick_conn_socket_ << "trick.var_pause()\n";
  trick_conn_socket_ << "trick.var_ascii()\n";
  trick_conn_socket_ << "trick.var_set_client_tag(\"ros2_control_" + info_.name + "\") \n";
  trick_conn_socket_ << "trick.var_cycle(\"" + std::to_string(1.0 / trick_variable_cycle_rate_) + "\")\n";

  for (const TrickData& trick_state_variable : trick_state_variables_)
  {
    const std::string& trick_var_name = trick_state_variable.trick_var_name;
    std::string request_ = "trick.var_add(\"" + trick_var_name + "\")\n";
    RCLCPP_INFO(rclcpp::get_logger("TrickSystem"), "adding trick variable with command: %s", request_.c_str());
    trick_conn_socket_ << request_;
  }

  incoming_data_thread_ = std::thread([this]() {
    while (rclcpp::ok())
    {
      if (this->get_state().label() == "active")
      {
        std::string incoming_msg;
        this->trick_conn_socket_ >> incoming_msg;
        std::vector<double> trick_data = trick_response_convert(incoming_msg);
        this->trick_data_incoming_buffer_.writeFromNonRT(trick_data);
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  });

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TrickSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (unsigned int i = 0; i < info_.joints.size(); i++)
  {
    auto& joint_info = info_.joints[i];
    for (unsigned int j = 0; j < joint_info.state_interfaces.size(); ++j)
    {
      double* state_variable_ptr_;
      std::string joint_interface_name = joint_info.state_interfaces[j].name;
      if (joint_interface_name == hardware_interface::HW_IF_POSITION)
      {
        state_variable_ptr_ = &joints_data_[i].joint_position;
      }
      if (joint_interface_name == hardware_interface::HW_IF_VELOCITY)
      {
        state_variable_ptr_ = &joints_data_[i].joint_velocity;
      }
      if (joint_interface_name == hardware_interface::HW_IF_ACCELERATION)
      {
        state_variable_ptr_ = &joints_data_[i].joint_acceleration;
      }
      if (joint_interface_name == hardware_interface::HW_IF_EFFORT)
      {
        state_variable_ptr_ = &joints_data_[i].joint_effort;
      }
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(joint_info.name, joint_interface_name, state_variable_ptr_));
      std::string trick_variable_name_param_key =
          JOINT_PARAM_STATE_INTERFACE_KEY + "/" + joint_interface_name + "/" + JOINT_PARAM_TRICK_VARIABLE_NAME_KEY;

      if (joint_info.parameters.find(trick_variable_name_param_key) == joint_info.parameters.end())
      {
        throw std::runtime_error("Missing parameter: " + trick_variable_name_param_key +
                                 ". Add it to joint parameters in ros2_control block, so that trick "
                                 "variable can be "
                                 "binded to "
                                 "ros2_control variable");
      }
      std::string trick_variable_name = joint_info.parameters[trick_variable_name_param_key];
      trick_state_variables_.emplace_back(TrickData(trick_variable_name, state_variable_ptr_));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TrickSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (unsigned int i = 0; i < info_.joints.size(); i++)
  {
    auto& joint_info = info_.joints[i];
    for (unsigned int j = 0; j < joint_info.command_interfaces.size(); ++j)
    {
      double* command_variable_ptr_;
      std::string joint_interface_name = joint_info.command_interfaces[j].name;
      if (joint_interface_name == hardware_interface::HW_IF_POSITION)
      {
        command_variable_ptr_ = &joints_data_[i].joint_position_cmd;
      }
      if (joint_interface_name == hardware_interface::HW_IF_VELOCITY)
      {
        command_variable_ptr_ = &joints_data_[i].joint_velocity_cmd;
      }
      if (joint_interface_name == hardware_interface::HW_IF_ACCELERATION)
      {
        command_variable_ptr_ = &joints_data_[i].joint_acceleration_cmd;
      }
      if (joint_interface_name == hardware_interface::HW_IF_EFFORT)
      {
        command_variable_ptr_ = &joints_data_[i].joint_effort_cmd;
      }
      // TODO(later) - parametrize it with joint params
      *command_variable_ptr_ = 0.0;
      command_interfaces.emplace_back(
          hardware_interface::CommandInterface(joint_info.name, joint_interface_name, command_variable_ptr_));
      std::string trick_variable_name_param_key =
          JOINT_PARAM_COMMAND_INTERFACE_KEY + "/" + joint_interface_name + "/" + JOINT_PARAM_TRICK_VARIABLE_NAME_KEY;

      if (joint_info.parameters.find(trick_variable_name_param_key) == joint_info.parameters.end())
      {
        throw std::runtime_error("Missing parameter: " + trick_variable_name_param_key +
                                 ". Add it to joint parameters in ros2_control block, so that trick "
                                 "variable can be "
                                 "binded to "
                                 "ros2_control variable");
      }
      std::string trick_variable_name = joint_info.parameters[trick_variable_name_param_key];
      trick_command_variables_.emplace_back(TrickData(trick_variable_name, command_variable_ptr_));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn TrickSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  trick_conn_socket_ << "trick.var_unpause()\n";
  // receive first message to  ensure connection works
  trick_conn_socket_.receive();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrickSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  trick_conn_socket_ << "trick.var_pause()\n";
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type TrickSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  const std::vector<double>& raw_doubles_from_trick = *trick_data_incoming_buffer_.readFromRT();
  if (raw_doubles_from_trick.size() != trick_state_variables_.size())
  {
    RCLCPP_FATAL(rclcpp::get_logger("TrickSystem"),
                 "size of data received from trick does not match number of "
                 "declared state variables. Expected: %d. "
                 "Got: %d",
                 trick_state_variables_.size(), raw_doubles_from_trick.size());
    return hardware_interface::return_type::OK;
  }
  for (int i = 0; i < raw_doubles_from_trick.size(); ++i)
  {
    double value_from_trick = raw_doubles_from_trick[i];
    double* const target_state_variable_ptr_ = trick_state_variables_[i].data_ptr;
    *target_state_variable_ptr_ = value_from_trick;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TrickSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  std::string command = "";
  for (const TrickData& trick_command_variable : trick_command_variables_)
  {
    command += trick_command_variable.trick_var_name + " = " + std::to_string(*trick_command_variable.data_ptr) + "\n";
  }
  trick_conn_socket_ << command;
  return hardware_interface::return_type::OK;
}

}  // namespace trick_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(trick_ros2_control::TrickSystem, hardware_interface::SystemInterface)
