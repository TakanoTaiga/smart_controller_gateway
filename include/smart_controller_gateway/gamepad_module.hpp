// Copyright 2023 Hakoroboken
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

#ifndef SMART_CONTROLLER_GATEWAY__GAMEPAD_MODULE_HPP_
#define SMART_CONTROLLER_GATEWAY__GAMEPAD_MODULE_HPP_

#include "smart_controller_gateway/network_data.hpp"
#include <remote_control_msgs/msg/gamepad.hpp>
#include <remote_control_msgs/msg/smart_ui.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gamepad_module
{
    remote_control_msgs::msg::Gamepad 
    gamepad_data_to_msg(const network_data::gamepad_rcv_data *gamepad_data_ptr_);

    remote_control_msgs::msg::SmartUI 
    smart_ui_data_to_msg(const network_data::gamepad_rcv_data *gamepad_data_ptr_);

    geometry_msgs::msg::Twist 
    gamepad_msg_to_twist_msg(const remote_control_msgs::msg::Gamepad & gamepad_msg);

} //namespace gamepad_module

#endif