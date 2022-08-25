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

#ifndef SPACE_ROS_MEMORY_ALLOCATION_DEMO__LISTENER_HPP_
#define SPACE_ROS_MEMORY_ALLOCATION_DEMO__LISTENER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "space_ros_memory_allocation_demo/visibility_control.h"
#include "space_ros_memory_allocation_demo/memory_allocator.hpp"

#include "expected_push_pop.hpp"

namespace space_ros_memory_allocation_demo
{

// TODO(wjwwood): Cannot use this until we fix https://github.com/ros2/rosidl/issues/566
// using String = std_msgs::msg::String_<MemoryAllocator>;
using String = std_msgs::msg::String;

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  SPACE_ROS_MEMORY_ALLOCATION_DEMO_PUBLIC
  explicit Listener(
    const rclcpp::NodeOptions & options,
    std::pmr::memory_resource * memory_resource = std::pmr::get_default_resource())
  : Node("listener", options)
  {
    RCUTILS_UNUSED(memory_resource);
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
      [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        this->get_node_base_interface()->get_context()->shutdown("done");
      };
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace space_ros_memory_allocation_demo

#endif  // SPACE_ROS_MEMORY_ALLOCATION_DEMO__LISTENER_HPP_
