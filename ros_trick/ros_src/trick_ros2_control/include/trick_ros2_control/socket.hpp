// Copyright 2024 Blazej Fiderek (xfiderek)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Largely inspired by
// https://github.com/nasa/trick/blob/master/trick_sims/SIM_billiards/models/graphics/cpp/Socket.cpp
#ifndef TRICK_ROS2_CONTROL__SOCKET_HPP_
#define TRICK_ROS2_CONTROL__SOCKET_HPP_

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#define SOCKET_BUF_SIZE 20480

class Socket
{
public:
  Socket();
  int init(std::string hostname, int port);

  int send(std::string message);
  int operator<<(std::string message);

  std::string receive();
  void operator>>(std::string& ret);

private:
  int _port;
  std::string _hostname;
  int _socket_fd;
  bool _initialized;
  char carry_on_buffer_[SOCKET_BUF_SIZE] = { '\0' };
  char trick_delimiter_ = '\n';
};

#endif
