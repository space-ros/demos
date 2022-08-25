// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef SPACE_ROS_MEMORY_ALLOCATION_DEMO__TALKER_HPP_
#define SPACE_ROS_MEMORY_ALLOCATION_DEMO__TALKER_HPP_

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "space_ros_memory_allocation_demo/visibility_control.h"
#include "space_ros_memory_allocation_demo/memory_allocator.hpp"

#include "expected_push_pop.hpp"

using namespace std::chrono_literals;

namespace space_ros_memory_allocation_demo
{

// TODO(wjwwood): Cannot use this until we fix https://github.com/ros2/rosidl/issues/566
// using String = std_msgs::msg::String_<MemoryAllocator>;
using String = std_msgs::msg::String;

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Talker : public rclcpp::Node
{
public:
  SPACE_ROS_MEMORY_ALLOCATION_DEMO_PUBLIC
  explicit Talker(
    const rclcpp::NodeOptions & options,
    std::pmr::memory_resource * memory_resource = std::pmr::get_default_resource())
  : Node("talker", options)
  {
    RCUTILS_UNUSED(memory_resource);
    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
        msg_ = std::make_unique<String>();
        msg_->data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        auto states = push_expectations({false, false, false, false});
        osrf_testing_tools_cpp::memory_tools::guaranteed_malloc("it's working");
        pub_->publish(std::move(msg_));
        pop_expectations(states);
        this->get_node_base_interface()->get_context()->shutdown("done");
      };
    // Create a publisher with a custom Quality of Service profile.
    // Uniform initialization is suggested so it can be trivially changed to
    // rclcpp::KeepAll{} if the user wishes.
    // (rclcpp::KeepLast(7) -> rclcpp::KeepAll() fails to compile)
    rclcpp::QoS qos(rclcpp::KeepLast{7});
    pub_ = this->create_publisher<String>("chatter", qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  size_t count_ = 1;
  std::unique_ptr<String> msg_;
  rclcpp::Publisher<String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace demo_nodes_cpp

#endif  // SPACE_ROS_MEMORY_ALLOCATION_DEMO__TALKER_HPP_
