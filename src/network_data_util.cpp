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

#include "smart_controller_gateway/network_data.hpp"

namespace network_data
{
    namespace util
    {
        network_data_util_status make_ip_response(
            const std::vector<int16_t> & node_ip ,
            network_data::ip_response & response
        ){
            if(node_ip.size() < 4){
                return network_data_util_status::error;
            }

            response.ip_addr_0 = node_ip.at(0);
            response.ip_addr_1 = node_ip.at(1);
            response.ip_addr_2 = node_ip.at(2);
            response.ip_addr_3 = node_ip.at(3);

            return network_data_util_status::complete;
        }
    } // namespace util
} // namespace network_data