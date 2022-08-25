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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "osrf_testing_tools_cpp/memory_tools/memory_tools.hpp"

#include "talker.hpp"
#include "common_main.hpp"

int
actual_main(int argc, char const * argv[], space_ros_memory_allocation_demo::MemoryAllocator & ma)
{
  using std::pmr::polymorphic_allocator;
  // Create a C-style allocator that wraps the pmr C++ allocator.
  polymorphic_allocator<void> pa(&ma);
  rcl_allocator_t rcl_allocator =
    rclcpp::allocator::get_rcl_allocator<void, polymorphic_allocator<void>>(pa);

  using rclcpp::Context;
  rclcpp::InitOptions io(rcl_allocator);
  auto context = std::allocate_shared<Context, polymorphic_allocator<Context>>(&ma);
  context->init(argc, argv, io);

  rclcpp::NodeOptions no(rcl_allocator);
  no.context(context);
  using space_ros_memory_allocation_demo::Talker;
  auto talker =
    std::allocate_shared<Talker, std::pmr::polymorphic_allocator<Talker>>(&ma, no, &ma);

  // TODO(wjwwood): use ma in executor memory_strategy
  rclcpp::ExecutorOptions eo;
  eo.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(eo);

  executor.add_node(talker);
  executor.spin();

  return 0;
}

int
main(int argc, char const * argv[])
{
  return common_main(argc, argv, actual_main);
}
