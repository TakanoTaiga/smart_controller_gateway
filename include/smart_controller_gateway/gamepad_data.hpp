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

#ifndef SMART_CONTROLLER_GATEWAY__GAMEPAD_DATA_HPP_
#define SMART_CONTROLLER_GATEWAY__GAMEPAD_DATA_HPP_

#include <string>

namespace gamepad_data
{
    struct gamepad_joystic_value
    {
        float x;
        float y;
        bool thumbstickButton;
    };
    
    struct gamepad_trigger_value
    {
        float value;
        bool button;
    };

    struct gamepad_dpad_value
    {
        bool up;
        bool down;
        bool left;
        bool right;
    };

    struct gamepad_button_value
    {
        bool x;
        bool y;
        bool a;
        bool b;
    };

    struct gamepad_value
    {
        gamepad_joystic_value left_joystic;
        gamepad_joystic_value right_joystic;

        gamepad_trigger_value left_trigger;
        gamepad_trigger_value right_trigger;

        gamepad_dpad_value dpad;
        gamepad_button_value button;

        bool left_shoulder_button;
        bool right_shoulder_button;
    };

    struct smart_ui_value
    {
        bool button_a;
        bool button_b;
        float slider;
    };
    

    struct parameter{
        std::string button_a_label;
        std::string button_b_label;
        std::string slider_label;
    };
} //namespace gamepad_data

#endif