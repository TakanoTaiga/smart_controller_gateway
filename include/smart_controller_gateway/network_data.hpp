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

#ifndef SMART_CONTROLLER_GATEWAY__NETWORK_DATA_HPP_
#define SMART_CONTROLLER_GATEWAY__NETWORK_DATA_HPP_

#include <sys/socket.h>
#include <netinet/in.h>
#include <vector>
#include "smart_controller_gateway/gamepad_data.hpp"

namespace network_data{
    struct smart_controller_network_data
    {   
        // File descriptor for the receiving socket
        int rcv_sock;

        // File descriptor for the sending socket
        int send_socket;

        // Address for receiving
        struct sockaddr_in addr;

        // Variable for temporarily storing the sender address
        struct sockaddr_in from_addr;

        // Address for receiving
        struct sockaddr_in send_addr;

        // Variable that stores the address of the application side
        socklen_t sin_size;

        // Node ip address vector
        std::vector<int16_t> node_ip;
    };

    struct ip_response
    {
        const uint8_t header_id = 75;
        int16_t ip_addr_0;
        int16_t ip_addr_1;
        int16_t ip_addr_2;
        int16_t ip_addr_3;
    };

    struct gamepad_rcv_data
    {
        uint8_t header_id;
        gamepad_data::gamepad_value gamepad_value;
        gamepad_data::smart_ui_value smart_ui_value;
    };

    struct parameter{
        std::string network_interface;
        bool is_publish_twist;
        bool is_publish_rc_rover;
    };

    namespace util
    {
        enum class network_data_util_status
        {
            complete,
            error,
        };

        network_data_util_status make_ip_response(
            const std::vector<int16_t> & node_ip ,
            network_data::ip_response & response
        );

    } // namespace util
} // namespace network_data

namespace NodeConnectionKey
{
    const uint8_t searchNode = 47;
    const uint8_t pingRequest = 98;
    const uint8_t pingResponse = 87;
    const uint8_t ipResponse = 75;
    const uint8_t nodeInfoRequest = 23;
    const uint8_t nodeInfoResponse = 25;
    const uint8_t gamepadValueRequest = 12;
} //namespace NodeConnectionKey

#endif