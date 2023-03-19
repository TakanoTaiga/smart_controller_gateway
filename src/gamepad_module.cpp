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

#include "smart_controller_gateway/gamepad_module.hpp"

namespace gamepad_module
{
    remote_control_msgs::msg::Gamepad 
    gamepad_data_to_msg(const network_data::gamepad_rcv_data *gamepad_data_ptr_)
    {
        auto gamepad_msg = remote_control_msgs::msg::Gamepad();
        gamepad_msg.left_joystic.x = gamepad_data_ptr_->gamepad_value.left_joystic.x;
        gamepad_msg.left_joystic.y = gamepad_data_ptr_->gamepad_value.left_joystic.y;
        gamepad_msg.left_joystic.thumbstick_button = gamepad_data_ptr_->gamepad_value.left_joystic.thumbstickButton;

        gamepad_msg.right_joystic.x = gamepad_data_ptr_->gamepad_value.right_joystic.x;
        gamepad_msg.right_joystic.y = gamepad_data_ptr_->gamepad_value.right_joystic.y;
        gamepad_msg.right_joystic.thumbstick_button = gamepad_data_ptr_->gamepad_value.right_joystic.thumbstickButton;

        gamepad_msg.left_trigger.value = gamepad_data_ptr_->gamepad_value.left_trigger.value;
        gamepad_msg.left_trigger.button = gamepad_data_ptr_->gamepad_value.left_trigger.button;

        gamepad_msg.right_trigger.value = gamepad_data_ptr_->gamepad_value.right_trigger.value;
        gamepad_msg.right_trigger.button = gamepad_data_ptr_->gamepad_value.right_trigger.button;

        gamepad_msg.dpad.up = gamepad_data_ptr_->gamepad_value.dpad.up;
        gamepad_msg.dpad.down = gamepad_data_ptr_->gamepad_value.dpad.down;
        gamepad_msg.dpad.left = gamepad_data_ptr_->gamepad_value.dpad.left;
        gamepad_msg.dpad.right = gamepad_data_ptr_->gamepad_value.dpad.right;

        gamepad_msg.button.x = gamepad_data_ptr_->gamepad_value.button.x;
        gamepad_msg.button.y = gamepad_data_ptr_->gamepad_value.button.y;
        gamepad_msg.button.a = gamepad_data_ptr_->gamepad_value.button.a;
        gamepad_msg.button.b = gamepad_data_ptr_->gamepad_value.button.b;

        gamepad_msg.left_shoulder_button = gamepad_data_ptr_->gamepad_value.left_shoulder_button;
        gamepad_msg.right_shoulder_button = gamepad_data_ptr_->gamepad_value.right_shoulder_button;

        return gamepad_msg;
    }

    remote_control_msgs::msg::SmartUI 
    smart_ui_data_to_msg(const network_data::gamepad_rcv_data *gamepad_data_ptr_)
    {
        auto smart_ui_msg = remote_control_msgs::msg::SmartUI();
        smart_ui_msg.button_a = gamepad_data_ptr_->smart_ui_value.button_a;
        smart_ui_msg.button_b = gamepad_data_ptr_->smart_ui_value.button_b;
        smart_ui_msg.slider = gamepad_data_ptr_->smart_ui_value.slider;
        return smart_ui_msg;
    }

    geometry_msgs::msg::Twist 
    gamepad_msg_to_twist_msg(const remote_control_msgs::msg::Gamepad & gamepad_msg){
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = gamepad_msg.left_joystic.x;
        twist_msg.linear.y = gamepad_msg.left_joystic.y;
        twist_msg.angular.z = gamepad_msg.right_joystic.x;
        return twist_msg;
    }

} //namespace gamepad_modue