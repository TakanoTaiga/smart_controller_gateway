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

#include <unistd.h>
#include <memory>
#include <vector>
#include <chrono>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <rclcpp/rclcpp.hpp>

#include "smart_controller_gateway/network_module.hpp"
#include "smart_controller_gateway/smart_controller_network.hpp"


namespace network_module
{
    std::string gethostname_str(){
        char hostname[MAX_HOST_NAME_CHARCTOR_SIZE];
        gethostname(hostname, sizeof(hostname));
        return std::string(hostname);
    }

    std::string get_ip_str(const std::string & network_interface)
    {
        int fd_gi;
        struct ifreq ifr_gi;

        fd_gi = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_gi == -1) {
            perror("socket");
            return "";
        }

        ifr_gi.ifr_addr.sa_family = AF_INET;

        strncpy(ifr_gi.ifr_name, network_interface.c_str(), IF_NAMESIZE - 1);
        if (ioctl(fd_gi, SIOCGIFADDR, &ifr_gi) == -1) {
            perror("ioctl");
            close(fd_gi);
            return "";
        }
        close(fd_gi);

        std::string ip_str = inet_ntoa(((struct sockaddr_in *)&ifr_gi.ifr_addr)->sin_addr);
        return ip_str;
    }

    std::vector<int16_t> get_ip_vec(const std::string & network_interface)
    {
        auto ip_str = get_ip_str(network_interface);
        std::vector<int16_t> ip_i16_vec;
        std::string addr;
        for(auto & char_ : ip_str){
            if(char_ != '.'){
                addr += char_;
            }else{
                ip_i16_vec.push_back(std::stoi(addr));
                addr = "";
            }
        }
        ip_i16_vec.push_back(std::stoi(addr));

        return ip_i16_vec;
    }


    void network_starter(network_data::smart_controller_network_data & network_data_)
    {
        network_data_.rcv_sock = socket(AF_INET, SOCK_DGRAM, 0);
        network_data_.send_socket = socket(AF_INET, SOCK_DGRAM, 0);

        network_data_.addr.sin_family = AF_INET;
        network_data_.send_addr.sin_family = AF_INET;
        network_data_.addr.sin_addr.s_addr = inet_addr("0.0.0.0");
        network_data_.addr.sin_port = htons(port);
        network_data_.send_addr.sin_port = htons(port);

        bind(network_data_.rcv_sock, (const struct sockaddr *)&network_data_.addr, sizeof(network_data_.addr));
        static const int val = 1;
        ioctl(network_data_.rcv_sock, FIONBIO, &val);
    }

    network_module_status receive_sock(
        network_data::smart_controller_network_data & network_data_ ,
        void * buffer_ptr_ ,
        const size_t buffer_size
    ){
        if(buffer_ptr_ == NULL){
            return network_module_status::failed;
        }

        if(recvfrom(network_data_.rcv_sock ,
                buffer_ptr_ ,
                buffer_size ,
                0 ,
                (struct sockaddr *)&network_data_.from_addr ,
                &network_data_.sin_size) < 0){
            return network_module_status::failed;
        }
        return network_module_status::ready;
    }
    
    void nw_send(
        network_data::smart_controller_network_data & network_data_ ,
        const void * send_item ,
        const size_t item_size
    ){
        if(send_item == NULL){return;}

        sendto(network_data_.send_socket,
            send_item,
            item_size,
            0,
            (struct sockaddr *)&network_data_.send_addr,
            sizeof(network_data_.send_addr));
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
}