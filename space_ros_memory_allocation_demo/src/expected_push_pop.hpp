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

#ifndef SPACE_ROS_MEMORY_ALLOCATION_DEMO__EXPECTED_PUSH_POP_HPP_
#define SPACE_ROS_MEMORY_ALLOCATION_DEMO__EXPECTED_PUSH_POP_HPP_

#include "osrf_testing_tools_cpp/memory_tools/memory_tools.hpp"

struct states
{
  bool malloc_expected;
  bool calloc_expected;
  bool realloc_expected;
  bool free_expected;
};

inline
void
set_expectations_from_states(states expectations)
{
  if (expectations.malloc_expected) {
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_end();
  } else {
    osrf_testing_tools_cpp::memory_tools::expect_no_malloc_begin();
  }
  if (expectations.calloc_expected) {
    osrf_testing_tools_cpp::memory_tools::expect_no_calloc_end();
  } else {
    osrf_testing_tools_cpp::memory_tools::expect_no_calloc_begin();
  }
  if (expectations.realloc_expected) {
    osrf_testing_tools_cpp::memory_tools::expect_no_realloc_end();
  } else {
    osrf_testing_tools_cpp::memory_tools::expect_no_realloc_begin();
  }
  if (expectations.free_expected) {
    osrf_testing_tools_cpp::memory_tools::expect_no_free_end();
  } else {
    osrf_testing_tools_cpp::memory_tools::expect_no_free_begin();
  }
}

inline
states
push_expectations(states new_states)
{
  states s;
  s.malloc_expected = osrf_testing_tools_cpp::memory_tools::malloc_expected();
  s.calloc_expected = osrf_testing_tools_cpp::memory_tools::calloc_expected();
  s.realloc_expected = osrf_testing_tools_cpp::memory_tools::realloc_expected();
  s.free_expected = osrf_testing_tools_cpp::memory_tools::free_expected();
  set_expectations_from_states(new_states);
  return s;
}

inline
void
pop_expectations(states previous_states)
{
  set_expectations_from_states(previous_states);
}

#endif  // SPACE_ROS_MEMORY_ALLOCATION_DEMO__EXPECTED_PUSH_POP_HPP_
