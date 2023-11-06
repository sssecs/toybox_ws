// Copyright 2021 ros2_control Development Team
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

#include "ros2_control_demo_hardware/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{
hardware_interface::return_type DiffBotSystemHardware::configure(
  const hardware_interface::HardwareInfo & info)
{

  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  this->wheel_param_l_.wheel_joint_name = info_.hardware_parameters["left_wheel_name"];
  this->wheel_param_r_.wheel_joint_name = info_.hardware_parameters["right_wheel_name"];
  this->serial_device_ = info_.hardware_parameters["serial_device"];
  this->timeout_ms_ = stoi(info_.hardware_parameters["timeout_ms"]);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      this->wheel_param_l_.wheel_joint_name, hardware_interface::HW_IF_POSITION, &this->wheel_param_l_.pos_act));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      this->wheel_param_l_.wheel_joint_name, hardware_interface::HW_IF_VELOCITY, &this->wheel_param_l_.velo_act));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      this->wheel_param_r_.wheel_joint_name, hardware_interface::HW_IF_POSITION, &this->wheel_param_r_.pos_act));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      this->wheel_param_r_.wheel_joint_name, hardware_interface::HW_IF_VELOCITY, &this->wheel_param_r_.velo_act));
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      this->wheel_param_l_.wheel_joint_name, hardware_interface::HW_IF_VELOCITY, &this->wheel_param_l_.velo_cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      this->wheel_param_r_.wheel_joint_name, hardware_interface::HW_IF_VELOCITY, &this->wheel_param_r_.velo_cmd));
  }

  return command_interfaces;
}

hardware_interface::return_type DiffBotSystemHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Starting ...please wait...");

  this->toybox_comms.connect(this->serial_device_, this->timeout_ms_);


  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Stopping ...please wait...");

  this->toybox_comms.disconnect();
  
  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;    
}

hardware_interface::return_type DiffBotSystemHardware::read()
{
  this->toybox_comms.read_rad_velo_pos(this->wheel_param_l_.velo_act,
                                       this->wheel_param_r_.velo_act,
                                       this->wheel_param_l_.pos_act,
                                       this->wheel_param_r_.pos_act);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_hardware::DiffBotSystemHardware::write()
{
  this->toybox_comms.send_rad_velo(this->wheel_param_l_.velo_cmd, this->wheel_param_r_.velo_cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::DiffBotSystemHardware, hardware_interface::SystemInterface)
