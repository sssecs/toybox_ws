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

#ifndef TOYBOX_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define TOYBOX_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "visibility_control.h"
#include "stm32_comms.h"

namespace toybox_hardware
{

struct WheelParam 
{
  std::string wheel_joint_name;
  double velo_act;
  double pos_act;
  double velo_cmd;
};

class DiffBotSystemHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  TOYBOX_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  TOYBOX_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TOYBOX_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TOYBOX_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  TOYBOX_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  TOYBOX_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  TOYBOX_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  Stm32Comms toybox_comms;
  WheelParam wheel_param_l_, wheel_param_r_;
  int16_t timeout_ms_;
  std::string serial_device_;
  
};

}  // namespace toybox_hardware

#endif  // TOYBOX_HARDWARE__DIFFBOT_SYSTEM_HPP_
