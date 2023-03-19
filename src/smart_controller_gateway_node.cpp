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

#include <memory>
#include <chrono>
#include <vector>
#include <cstdlib>

#include "smart_controller_gateway/smart_controller_gateway_node.hpp"
#include "smart_controller_gateway/smart_controller_network.hpp"
#include "smart_controller_gateway/gamepad_module.hpp"

namespace smart_controller_gateway
{
    SmartControllerGatewayNode::SmartControllerGatewayNode(const rclcpp::NodeOptions &node_option)
        : rclcpp::Node("smart_controller_gateway", node_option)
    {   
        // declare network parameter
        {
            network_param.network_interface = this->declare_parameter<std::string>("network_interface" , "eno1");
            network_param.is_pubulish_twist = this->declare_parameter<bool>("is_pubulish_twist" , false);
            network_param.is_pubulish_rc_rover = this->declare_parameter<bool>("is_pubulish_rc_rover" , false);
        }

        // declare gamepad parameter
        {
            gamepat_param.button_a_label = this->declare_parameter<std::string>("button_a_label" , "buttaon a");
            gamepat_param.button_b_label = this->declare_parameter<std::string>("button_b_label" , "buttaon b");
            gamepat_param.slider_label = this->declare_parameter<std::string>("slider_label" , "slider");
        }


        pub_gamepad_ = create_publisher<remote_control_msgs::msg::Gamepad>("~/output/gamepad", 1);

        network_module::network_starter(network_data_);
        network_data_.node_ip = network_module::get_ip_vec(network_param.network_interface);

        buffer_ = malloc(256);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]()
        {
            if(network_module::receive_sock(network_data_ , buffer_ , 256 ) != network_module::network_module_status::ready){return;}
            timer_callback(buffer_);
        });
    }

    void SmartControllerGatewayNode::timer_callback(const void *buffer_ptr_)
    {
        const auto header_ptr = (uint8_t *)buffer_ptr_;
        RCLCPP_INFO(get_logger(), "header id:%d", *header_ptr); // debug only

        if (*header_ptr == NodeConnectionKey::searchNode)
        {
            network_data_.send_addr.sin_addr = network_data_.from_addr.sin_addr;

            auto send_data = network_data::ip_response();
            network_data::util::make_ip_response(network_data_.node_ip, send_data);
            network_module::nw_send(network_data_, &send_data, sizeof(send_data));
            return;
        }

        if (*header_ptr == NodeConnectionKey::pingRequest)
        {
            const uint8_t header_id = NodeConnectionKey::pingResponse;
            network_module::nw_send(network_data_, &header_id, sizeof(header_id));
            return;
        }

        if (*header_ptr == NodeConnectionKey::nodeInfoRequest)
        {
            uint8_t send_buf_[128] = {0};
            send_buf_[0] = NodeConnectionKey::nodeInfoResponse;
            auto hostname = network_module::gethostname_str().c_str();
            memcpy(&send_buf_[1], hostname, strlen(hostname));

            auto button_a = gamepat_param.button_a_label.c_str();
            memcpy(&send_buf_[17], button_a, strlen(button_a));
            auto button_b = gamepat_param.button_b_label.c_str();
            memcpy(&send_buf_[33], button_b, strlen(button_b));
            auto slider = gamepat_param.slider_label.c_str();
            memcpy(&send_buf_[49], slider, strlen(slider));

            network_module::nw_send(network_data_, &send_buf_, sizeof(send_buf_));
            return;
        }

        if (*header_ptr == NodeConnectionKey::gamepadValueRequest)
        {
            pub_gamepad_->publish(
                gamepad_modue::gamepad_data_to_msg(
                    (network_data::gamepad_rcv_data *)buffer_ptr_
                )
            );
            return;
        }

        RCLCPP_WARN(get_logger(), "Received unknown header id: %d", *header_ptr);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(smart_controller_gateway::SmartControllerGatewayNode)