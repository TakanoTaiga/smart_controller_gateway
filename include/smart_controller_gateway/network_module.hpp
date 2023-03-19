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

#ifndef SMART_CONTROLLER_GATEWAY__NETWORK_MODULE_HPP_
#define SMART_CONTROLLER_GATEWAY__NETWORK_MODULE_HPP_

#include <string>
#include <vector>
#include "smart_controller_gateway/network_data.hpp"

#define MAX_HOST_NAME_CHARCTOR_SIZE 256

namespace network_module
{
    /**
    * @brief network status. use in receive_sock.
    */
    enum class network_module_status
    {
        ready,
        failed,
    };

    /**
    * @brief get host name
    */
    std::string gethostname_str();

    /**
    * @brief get ip by nic
    */
    std::string get_ip_str(const std::string & nic);

    /**
    * @brief get ip by nic
    */
    std::vector<int16_t> get_ip_vec(const std::string & nic);

    /**
    * @brief initilaize network service
    */
    void network_starter(network_data::smart_controller_network_data & network_data_);

    /**
    * @brief socket message capture
    */
    network_module_status receive_sock(
        network_data::smart_controller_network_data & network_data_ ,
        void * buffer_ptr_ ,
        const size_t buffer_size
    );

    /**
    * @brief send data to app
    */
    void nw_send(
        network_data::smart_controller_network_data & network_data_ ,
        const void * send_item ,
        const size_t item_size
    );

    constexpr uint16_t port = 64201;
}

#endif