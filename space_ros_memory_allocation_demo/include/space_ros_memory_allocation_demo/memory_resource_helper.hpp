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

#ifndef SPACE_ROS_MEMORY_ALLOCATION_DEMO__MEMORY_RESOURCE_HELPER_HPP_
#define SPACE_ROS_MEMORY_ALLOCATION_DEMO__MEMORY_RESOURCE_HELPER_HPP_

/// \file This header tries to provide a consistent std::pmr namespace even
///   on platforms where it isn't provided, like with llvm, see:
///     https://reviews.llvm.org/D89057 (not merged as of August 2022)

#if !defined(__has_include)
#define __has_include(x) 0
#endif

#if __has_include(<memory_resource>)

// Just include <memory_resource> and we're done.
#include <memory_resource>

#elif __has_include(<experimental/memory_resource>)

#include <algorithm>
#include <cassert>
// Include <experimental/memory_resource>, move out of experimental namespace, fill gaps.
#include <experimental/memory_resource>

#include "rcutils/macros.h"

namespace std::pmr
{

using namespace std::experimental::pmr;

/// Implementation of the std::pmr::monotonic_buffer_resource.
/**
 * Definitely a suboptimal implementation, but good enough for development on
 * platforms without full support.
 *
 * From https://en.cppreference.com/w/cpp/memory/monotonic_buffer_resource:
 *
 * The class std::pmr::monotonic_buffer_resource is a special-purpose memory
 * resource class that releases the allocated memory only when the resource is
 * destroyed.
 * It is intended for very fast memory allocations in situations where memory is
 * used to build up a few objects and then is released all at once.
 *
 * monotonic_buffer_resource can be constructed with an initial buffer.
 * If there is no initial buffer, or if the buffer is exhausted, additional
 * buffers are obtained from an upstream memory resource supplied at
 * construction.
 * The size of buffers obtained follows a geometric progression.
 *
 * monotonic_buffer_resource is not thread-safe.
 */
class monotonic_buffer_resource : public memory_resource
{
public:
  /**
   * https://en.cppreference.com/w/cpp/memory/monotonic_buffer_resource/monotonic_buffer_resource
   *
   * monotonic_buffer_resource();                           (1) (since C++17)
   *
   * explicit
   * monotonic_buffer_resource(
   *   std::pmr::memory_resource * upstream);               (2) (since C++17)
   *
   * explicit
   * monotonic_buffer_resource(std::size_t initial_size);   (3) (since C++17)
   *
   * monotonic_buffer_resource(
   *   std::size_t initial_size,
   *   std::pmr::memory_resource * upstream);               (4) (since C++17)
   *
   * monotonic_buffer_resource(
   *   void * buffer,
   *   std::size_t buffer_size);                            (5) (since C++17)
   *
   * monotonic_buffer_resource(
   *   void * buffer,
   *   std::size_t buffer_size,
   *   std::pmr::memory_resource * upstream);               (6) (since C++17)
   *
   * monotonic_buffer_resource(
   *   const monotonic_buffer_resource &) = delete;         (7) (since C++17)
   *
   * Constructs a monotonic_buffer_resource.
   * The constructors not taking an upstream memory resource pointer use
   * the return value of std::pmr::get_default_resource as the upstream
   * memory resource.
   *
   *   1-2) Sets the current buffer to null and the next buffer size to
   *        an implementation-defined size.
   *   3-4) Sets the current buffer to null and the next buffer size to
   *        a size no smaller than initial_size.
   *   5-6) Sets the current buffer to buffer and the next buffer size to
   *        buffer_size (but not less than 1).
   *        Then increase the next buffer size by an implementation-defined
   *        growth factor (which does not have to be integral).
   *   7) Copy constructor is deleted.
   *
   * Parameters:
   *
   * - upstream: the upstream memory resource to use;
   *             must point to a valid memory resource
   * - initial_size: the minimum size of the first buffer to allocate;
   *                 must be greater than zero
   * - buffer: the initial buffer to use
   * - buffer_size: the size of the initial buffer;
   *                cannot be greater than the number of bytes in buffer
   */

  // (1)
  monotonic_buffer_resource() noexcept
  : monotonic_buffer_resource(get_default_resource())
  {}

  // (2)
  monotonic_buffer_resource(std::pmr::memory_resource * upstream) noexcept
  RCUTILS_NONNULL
  : upstream_(upstream)
  {
#if !RCUTILS_HAS_NONNULL
    assert(nullptr != upstream);
#endif
  }

  // (3)
  monotonic_buffer_resource(std::size_t initial_size) noexcept
  : monotonic_buffer_resource(initial_size, get_default_resource())
  {}

  // (4)
  monotonic_buffer_resource(
    std::size_t initial_size,
    std::pmr::memory_resource * upstream) noexcept
  RCUTILS_NONNULL_ARGS(3)
  : next_buffer_size_{initial_size},
    upstream_{upstream}
  {
    assert(initial_size > 0);
#if !RCUTILS_HAS_NONNULL
    assert(nullptr != upstream);
#endif
  }

  // (5)
  monotonic_buffer_resource(void * buffer, std::size_t buffer_size) noexcept
  : monotonic_buffer_resource(buffer, buffer_size, get_default_resource())
  {}

  // (6)
  monotonic_buffer_resource(
    void * buffer,
    std::size_t buffer_size,
    std::pmr::memory_resource * upstream)
  RCUTILS_NONNULL_ARGS(4)
  : buffer_{buffer},
    size_{buffer_size},
    next_buffer_size_{next_buffer_size(buffer_size)},
    upstream_{upstream},
    original_buffer_{buffer},
    original_size_{buffer_size}
  {
    assert(nullptr != buffer || 0 == buffer_size);
#if !RCUTILS_HAS_NONNULL
    assert(nullptr != upstream);
#endif
  }

  // (7)
  monotonic_buffer_resource(const monotonic_buffer_resource &) = delete;

  /// Destructor.
  /**
   * https://en.cppreference.com/w/cpp/memory/monotonic_buffer_resource/%7Emonotonic_buffer_resource
   *
   * virtual ~monotonic_buffer_resource();                        (since C++17)
   *
   * Destroys a monotonic_buffer_resource.
   *
   * Deallocates all memory owned by this resource by calling this->release().
   */
  virtual
  ~monotonic_buffer_resource()
  {
    this->release();
  }

  monotonic_buffer_resource &
  operator=(const monotonic_buffer_resource &) = delete;

  /// Release all memory owned.
  /**
   * https://en.cppreference.com/w/cpp/memory/monotonic_buffer_resource/release
   *
   * Releases all allocated memory by calling the deallocate function on the
   * upstream memory resource as necessary.
   * Resets current buffer and next buffer size to their initial values at
   * construction.
   *
   * Memory is released back to the upstream resource even if deallocate has not
   * been called for some of the allocated blocks.
   */
  void
  release() noexcept
  {
    if (buffer_) {
      this->release_buffers();
    }

    // Reset to the original state.
    buffer_ = original_buffer_;
    if (buffer_) {
      size_ = original_size_;
      next_buffer_size_ = next_buffer_size(original_size_);
    } else {
      size_ = 0;
      next_buffer_size_ = original_size_;
    }
  }

  memory_resource *
  upstream_resource() const noexcept
  RCUTILS_RETURNS_NONNULL
  {
    return upstream_;
  }

private:
  virtual
  void *
  do_allocate(size_t bytes, size_t alignment) override
  {
    if (RCUTILS_UNLIKELY(0 == bytes)) {
      bytes = 1;  // This ensures that a new pointer is returned when bytes = 0
    }

    void * result = std::align(alignment, bytes, buffer_, size_);
    if (RCUTILS_UNLIKELY(nullptr == result)) {
      this->new_buffer(bytes, alignment);
      result = buffer_;
    }
    buffer_ = static_cast<uint8_t *>(buffer_) + bytes;
    size_ -= bytes;

    return result;
  }

  virtual
  void
  do_deallocate(void * p, size_t bytes, size_t alignment) override
  {
    (void) p;
    (void) bytes;
    (void) alignment;
  }

  virtual
  bool
  do_is_equal(const memory_resource & other) const noexcept override
  {
    return this == &other;
  }

  class LinkedListAllocationNode
  {
    friend monotonic_buffer_resource;

    LinkedListAllocationNode(
      std::size_t size,
      std::size_t alignment,
      LinkedListAllocationNode * next) noexcept
    : size_{size},
      alignment_{alignment},
      next_{next}
    {}

    // static
    // std::size_t
    // align_to(std::size_t unaligned_value, std::size_t alignment)
    // {
    //   auto mask = alignment - 1;
    //   return unaligned_value + (-unaligned_value & mask);
    // }

    static
    std::pair<void *, std::size_t>
    allocate(
      std::size_t size,
      std::size_t alignment,
      memory_resource * resource,
      LinkedListAllocationNode ** head)
    {
      // Allocate space for the requested size and this Node, with the alignment.
      void * pointer = resource->allocate(size + sizeof(LinkedListAllocationNode), alignment);

      // Placement new a Node at the end of the memory, replacing the head at the same time.
      void * back = static_cast<uint8_t *>(pointer) + size - sizeof(LinkedListAllocationNode);
      *head = ::new(back) LinkedListAllocationNode(size, alignment, *head);

      return {pointer, size - sizeof(LinkedListAllocationNode)};
    }

    static
    LinkedListAllocationNode *
    deallocate(
      LinkedListAllocationNode * node,
      memory_resource * resource)
    {
      LinkedListAllocationNode * next = node->next_;
      // |memory_location ...|node ...|
      // ^........size.......^
      void * memory_location = node - node->size_;
      resource->deallocate(memory_location, node->size_, node->alignment_);
      return next;
    }

    std::size_t size_;
    std::size_t alignment_;
    LinkedListAllocationNode * next_;
  };

  void
  release_buffers() noexcept
  {
    LinkedListAllocationNode * next = head_;
    while (nullptr != next) {
      next = LinkedListAllocationNode::deallocate(next, upstream_);
    }
    head_ = nullptr;
  }

  void
  new_buffer(std::size_t bytes, std::size_t alignment)
  {
    auto p = LinkedListAllocationNode::allocate(
      std::max(bytes, next_buffer_size_),
      std::max(alignment, sizeof(std::max_align_t)),
      upstream_,
      &head_);
    buffer_ = p.first;
    size_ = p.second;
    next_buffer_size_ = next_buffer_size(next_buffer_size_);
  }

  static
  std::size_t
  next_buffer_size(std::size_t buffer_size) noexcept
  {
    if (RCUTILS_UNLIKELY(buffer_size == 0)) {
      buffer_size = 1;
    }
    return buffer_size * k_common_ratio_;
  }

  /// Arbitrary default next buffer size, which is allowed by the std.
  static constexpr std::size_t k_default_next_buffer_size_ = 128 * sizeof(void *);
  /// Common ratio for the geometry progression.
  static constexpr std::size_t k_common_ratio_ = 2;

  void * buffer_ = nullptr;
  // void * pointer_ = nullptr;
  std::size_t size_ = 0;
  std::size_t next_buffer_size_ = k_default_next_buffer_size_;

  // Initial values set by constructors, used in release().
  std::pmr::memory_resource * upstream_;
  void * original_buffer_ = nullptr;
  std::size_t original_size_ = next_buffer_size_;

  // Linked list of allocations.
  LinkedListAllocationNode * head_;
};

struct pool_options
{
  size_t max_blocks_per_chunk = 0;

  size_t largest_required_pool_block = 0;
};

// TODO(wjwwood): actually finish implementation of this class

class unsynchronized_pool_resource : public memory_resource
{
public:
  RCUTILS_NONNULL
  unsynchronized_pool_resource(
    const pool_options & opts,
    memory_resource * upstream);

  unsynchronized_pool_resource()
  : unsynchronized_pool_resource(pool_options(), get_default_resource())
  {}

  RCUTILS_NONNULL
  explicit
  unsynchronized_pool_resource(memory_resource * upstream)
  : unsynchronized_pool_resource(pool_options(), upstream)
  {}

  explicit
  unsynchronized_pool_resource(const pool_options & opts)
  : unsynchronized_pool_resource(opts, get_default_resource()) {}

  unsynchronized_pool_resource(const unsynchronized_pool_resource &) = delete;

  virtual ~unsynchronized_pool_resource();

  unsynchronized_pool_resource &
  operator=(const unsynchronized_pool_resource &) = delete;

  void release();

  RCUTILS_NONNULL
  memory_resource *
  upstream_resource() const noexcept
  {
    return upstream_;
  }

  pool_options
  options() const noexcept
  {
    return pool_options_;
  }

protected:
  void *
  do_allocate(size_t __bytes, size_t __alignment) override;

  void
  do_deallocate(void * __p, size_t __bytes, size_t __alignment) override;

  bool
  do_is_equal(const memory_resource & __other) const noexcept override
  {
    return this == &__other;
  }

private:
  pool_options pool_options_;
  memory_resource * upstream_;

};

// TODO(wjwwood): fill in additional missing classes

}  // namespace std::pmr

#else

#error "no complete or partial <memory_resource> available"

#endif

#endif  // SPACE_ROS_MEMORY_ALLOCATION_DEMO__MEMORY_RESOURCE_HELPER_HPP_
