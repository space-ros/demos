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

#include <cstddef>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "osrf_testing_tools_cpp/memory_tools/memory_tools.hpp"

#include "talker.hpp"
#include "common_main.hpp"

template<typename MemoryAllocatorT>
int
actual_main(int argc, char const * argv[], MemoryAllocatorT & ma)
{
  using std::pmr::polymorphic_allocator;
  // Create a C-style allocator that wraps the pmr C++ allocator.
  polymorphic_allocator<std::byte> pa(&ma);
  rcl_allocator_t rcl_allocator =
    rclcpp::allocator::get_rcl_allocator<std::byte, polymorphic_allocator<std::byte>>(pa);

  // EXPECT_NO_MEMORY_OPERATIONS_BEGIN();
  using rclcpp::Context;
  rclcpp::InitOptions io(rcl_allocator);
  auto context = std::allocate_shared<Context, polymorphic_allocator<Context>>(pa);
  context->init(argc, argv, io);
  // EXPECT_NO_MEMORY_OPERATIONS_END();

  // EXPECT_NO_MEMORY_OPERATIONS_BEGIN();
  rclcpp::NodeOptions no(rcl_allocator);
  no.context(context);
  no.enable_topic_statistics(true);
  using space_ros_memory_allocation_demo::Talker;
  auto talker =
    std::allocate_shared<Talker, std::pmr::polymorphic_allocator<Talker>>(pa, no, &ma);
  // EXPECT_NO_MEMORY_OPERATIONS_END();

  // TODO(wjwwood): use ma in executor memory_strategy
  rclcpp::ExecutorOptions eo;
  eo.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(eo);

  // EXPECT_NO_MEMORY_OPERATIONS_BEGIN();
  executor.add_node(talker);
  // EXPECT_NO_MEMORY_OPERATIONS_END();
  // EXPECT_NO_MEMORY_OPERATIONS_BEGIN();
  executor.spin();
  // EXPECT_NO_MEMORY_OPERATIONS_END();

  return 0;
}

int
main(int argc, char const * argv[])
{
#if 0
  // using AllocatorT = MultiArena::UnsynchronizedArenaResource<16, 65536>;
  // AllocatorT a;

  using AllocatorT = MemoryAllocator;
  std::array<std::uint8_t, 65536*3> buffer;
  MemoryAllocator a(buffer.data(), buffer.size());

  using std::pmr::polymorphic_allocator;
  polymorphic_allocator<std::byte> pa(&a);
  std::basic_string<char, std::char_traits<char>, decltype(pa)> s(1024, 'a', pa);
  printf("%s\n", s.c_str());

  using PolymorphicAllocatorT = std::pmr::polymorphic_allocator<std::byte>;
  using String = std_msgs::msg::String_<PolymorphicAllocatorT>;
  auto msg_s = std::allocate_shared<String>(pa, pa);
  msg_s->data = s;
  printf("%s\n", msg_s->data.c_str());

  return 0;
#else
  // using AllocatorT = MultiArena::UnsynchronizedArenaResource<16, 65536>;
  using AllocatorT = MemoryAllocator;
  return common_main<AllocatorT>(argc, argv, actual_main<AllocatorT>);
#endif
}
