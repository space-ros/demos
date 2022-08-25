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

#ifndef SPACE_ROS_MEMORY_ALLOCATION_DEMO__MEMORY_ALLOCATOR_HPP_
#define SPACE_ROS_MEMORY_ALLOCATION_DEMO__MEMORY_ALLOCATOR_HPP_

#include <array>
#include <string>
#include <cassert>

// Provides std::pmr, accounting for differences in different platforms.
#include "space_ros_memory_allocation_demo/memory_resource_helper.hpp"

namespace space_ros_memory_allocation_demo
{

const unsigned int SMALL_MEM = 128;
const unsigned int MEDIUM_MEM = 4096;
const unsigned int LARGE_MEM = 32768;

template<size_t N>
using MEMORY_BUFFER = std::array<std::uint8_t, N>;

template<size_t N>
using PREALLOCATE = std::array<std::uint8_t, N>;


class MemoryAllocator : public std::pmr::memory_resource
{
public:
  MemoryAllocator(
    // std::string name,
    unsigned char * buffer,
    size_t buffer_size)
  : // m_name(std::move(name)),
    monotonic(buffer, buffer_size, std::pmr::null_memory_resource()),
    memory_pool(&monotonic)
  {}

private:
  // std::string m_name;
  // std::pmr::memory_resource * m_upstream;

  // Declare memory in the class stack
  std::pmr::monotonic_buffer_resource monotonic;
  std::pmr::unsynchronized_pool_resource memory_pool;

  void *
  do_allocate(std::size_t bytes, std::size_t alignment) override
  {
    auto result = memory_pool.allocate(bytes, alignment);
    return result;
  }

  void
  do_deallocate(void * p, std::size_t bytes, std::size_t alignment) override
  {
    memory_pool.deallocate(p, bytes, alignment);
  }

  bool
  do_is_equal(const memory_resource & other) const noexcept override
  {
    return this == &other;
  }
};

}  // namespace space_ros_memory_allocation_demo

#endif  // SPACE_ROS_MEMORY_ALLOCATION_DEMO__MEMORY_ALLOCATOR_HPP_
