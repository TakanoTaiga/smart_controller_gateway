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

#ifndef SMART_CONTROLLER_GATEWAY__SMART_CONTROLLER_GATEWAY_NODE_HPP_
#define SMART_CONTROLLER_GATEWAY__SMART_CONTROLLER_GATEWAY_NODE_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <remote_control_msgs/msg/gamepad.hpp>
#include <remote_control_msgs/msg/smart_ui.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "smart_controller_gateway/network_data.hpp"
#include "smart_controller_gateway/gamepad_data.hpp"

namespace smart_controller_gateway
{

class SmartControllerGatewayNode : public rclcpp::Node
{
public:
    explicit SmartControllerGatewayNode(const rclcpp::NodeOptions & node_options);

private:
    rclcpp::Publisher<remote_control_msgs::msg::Gamepad>::SharedPtr pub_gamepad_;
    rclcpp::Publisher<remote_control_msgs::msg::SmartUI>::SharedPtr pub_smart_ui_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback(const void* buffer_ptr_);

    void* buffer_;
    
    network_data::smart_controller_network_data network_data_;

    network_data::parameter network_param;
    gamepad_data::parameter gamepad_param;

};
} // namespace smart_controller_gateway

#endif