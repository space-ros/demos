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

#include "trick_ros2_control/socket.hpp"

Socket::Socket() : _initialized(false)
{
}

int Socket::init(std::string hostname, int port)
{
  _hostname = hostname;
  _port = port;
  _socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (_socket_fd < 0)
  {
    std::cout << "Socket connection failed" << std::endl;
    return -1;
  }

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);

  if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
  {
    std::cout << "Invalid address/ Address not supported" << std::endl;
    return -1;
  }

  if (connect(_socket_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
  {
    std::cout << "Connection failed" << std::endl;
    return -1;
  }

  _initialized = true;
  return 0;
}

int Socket::send(std::string message)
{
  int success = ::send(_socket_fd, message.c_str(), message.size(), 0);
  if (success < static_cast<int>(message.size()))
  {
    std::cout << "Failed to send message" << std::endl;
  }
  return success;
}

int Socket::operator<<(std::string message)
{
  return send(message);
}

std::string Socket::receive()
{
  char buffer[SOCKET_BUF_SIZE];
  int numBytes = read(_socket_fd, buffer, SOCKET_BUF_SIZE);
  if (numBytes < 0)
  {
    throw std::runtime_error("Failed to read from socket\n");
  }
  else if (numBytes < SOCKET_BUF_SIZE)
  {
    buffer[numBytes] = '\0';
  }

  std::string string_buffer = std::string(carry_on_buffer_) + std::string(buffer);
  int last_newline_idx = string_buffer.rfind(trick_delimiter_, string_buffer.size() - 1);
  int second_last_newline_idx = string_buffer.rfind(trick_delimiter_, string_buffer.size() - 2);

  // it means that we can clear carry on buffer
  if (last_newline_idx == string_buffer.size() - 1)
  {
    carry_on_buffer_[0] = '\0';
  }
  // in this case we need to store the remaining characters in the carry on
  // buffer
  else
  {
    std::strcpy(carry_on_buffer_, string_buffer.substr(last_newline_idx + 1).c_str());
  }

  // Always return the latest message
  return string_buffer.substr(second_last_newline_idx + 1, last_newline_idx - second_last_newline_idx - 1);
}

void Socket::operator>>(std::string& ret)
{
  ret = receive();
}
