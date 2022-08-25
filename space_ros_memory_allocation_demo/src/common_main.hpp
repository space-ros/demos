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

#ifndef SPACE_ROS_MEMORY_ALLOCATION_DEMO__COMMON_MAIN_HPP_
#define SPACE_ROS_MEMORY_ALLOCATION_DEMO__COMMON_MAIN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "osrf_testing_tools_cpp/memory_tools/memory_tools.hpp"

#include "space_ros_memory_allocation_demo/memory_allocator.hpp"

using space_ros_memory_allocation_demo::MemoryAllocator;

int
common_main(
  int argc,
  char const * argv[],
  std::function<int(int, char const *[], MemoryAllocator &)> actual_main)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::array<std::uint8_t, 65536> buffer;
  MemoryAllocator ma(buffer.data(), buffer.size());

  using osrf_testing_tools_cpp::memory_tools::MemoryToolsService;
  auto callback_factory = [](std::string msg, bool should_trace) {
      return [msg, should_trace](MemoryToolsService & service) {
               // filter out things that come from below the rmw layer
               auto st = service.get_stack_trace();
               // const std::regex pattern("/?librmw_\\.");
               const std::regex is_fastrtps("fastrtps");
               if (st && st->matches_any_object_filename(is_fastrtps)) {
                 service.ignore();
                 return;
               }
               // printf("%s\n", msg.c_str());
               if (should_trace) {
                 service.print_backtrace();
               }
             };
    };

  osrf_testing_tools_cpp::memory_tools::initialize();

  // osrf_testing_tools_cpp::memory_tools::enable_monitoring_in_all_threads();
  osrf_testing_tools_cpp::memory_tools::enable_monitoring();
  if (!osrf_testing_tools_cpp::memory_tools::is_working()) {
    printf("memory_tools not working\n");
  }
  osrf_testing_tools_cpp::memory_tools::disable_monitoring_in_all_threads();

  osrf_testing_tools_cpp::memory_tools::on_unexpected_malloc(callback_factory("malloc", true));
  osrf_testing_tools_cpp::memory_tools::on_unexpected_calloc(callback_factory("calloc", true));
  osrf_testing_tools_cpp::memory_tools::on_unexpected_realloc(callback_factory("realloc", true));
  // osrf_testing_tools_cpp::memory_tools::on_unexpected_free(callback_factory("free", true));

  osrf_testing_tools_cpp::memory_tools::enable_monitoring();

  return actual_main(argc, argv, ma);
}

#endif  // SPACE_ROS_MEMORY_ALLOCATION_DEMO__COMMON_MAIN_HPP_
