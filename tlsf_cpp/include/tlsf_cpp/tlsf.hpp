// Copyright 2015 Open Source Robotics Foundation, Inc.
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

// C++ wrapper for Miguel Masmano Tello's implementation of the TLSF memory allocator
// Implements the allocator_traits template

#ifndef TLSF_CPP__TLSF_HPP_
#define TLSF_CPP__TLSF_HPP_

#include <cstring>
#include <iostream>
#include <memory>
#include <new>
#include <stdexcept>

#include "rcl/allocator.h"
#include "rclcpp/allocator/allocator_common.hpp"
#include "tlsf/tlsf.h"

template<typename T, size_t DefaultPoolSize = 1024 * 1024>
struct tlsf_heap_allocator
{
  // Needed for std::allocator_traits
  using value_type = T;

  explicit tlsf_heap_allocator(size_t size)
  : memory_pool(nullptr), pool_size(size)
  {
    initialize(size);
  }

  // Needed for std::allocator_traits
  tlsf_heap_allocator()
  : memory_pool(nullptr)
  {
    initialize(DefaultPoolSize);
  }

  // Needed for std::allocator_traits
  template<typename U>
  tlsf_heap_allocator(const tlsf_heap_allocator<U> & alloc)
  : memory_pool(alloc.memory_pool), pool_size(alloc.pool_size)
  {
  }

  template<typename U, size_t OtherDefaultSize>
  tlsf_heap_allocator(const tlsf_heap_allocator<U> & alloc)
  : memory_pool(alloc.memory_pool), pool_size(alloc.pool_size)
  {
  }

  size_t initialize(size_t size)
  {
    pool_size = size;
    if (!memory_pool) {
      auto memory = new char[pool_size];
      memset(memory, 0, pool_size);
      init_memory_pool(pool_size, memory);
      memory_pool = std::shared_ptr<char>(
        memory,
        [](char * pool) {
          destroy_memory_pool(pool);
          delete[] pool;
        });
    }
    return pool_size;
  }

  ~tlsf_heap_allocator()
  {
  }

  // Needed for std::allocator_traits
  T * allocate(size_t size)
  {
    T * ptr = static_cast<T *>(tlsf_malloc(size * sizeof(T)));
    if (ptr == NULL && size > 0) {
      throw std::bad_alloc();
    }
    return ptr;
  }

  // Needed for std::allocator_traits
  void deallocate(T * ptr, size_t)
  {
    tlsf_free(ptr);
  }

  template<typename U>
  struct rebind
  {
    typedef tlsf_heap_allocator<U> other;
  };

  std::shared_ptr<char> memory_pool;
  size_t pool_size;
};

// Needed for std::allocator_traits
template<typename T, typename U>
constexpr bool operator==(
  const tlsf_heap_allocator<T> & a,
  const tlsf_heap_allocator<U> & b) noexcept
{
  return a.memory_pool == b.memory_pool;
}

// Needed for std::allocator_traits
template<typename T, typename U>
constexpr bool operator!=(
  const tlsf_heap_allocator<T> & a,
  const tlsf_heap_allocator<U> & b) noexcept
{
  return a.memory_pool != b.memory_pool;
}

template<typename T, typename U, size_t X, size_t Y>
constexpr bool operator==(
  const tlsf_heap_allocator<T, X> & a,
  const tlsf_heap_allocator<U, Y> & b) noexcept
{
  return a.memory_pool == b.memory_pool;
}

// Needed for std::allocator_traits
template<typename T, typename U, size_t X, size_t Y>
constexpr bool operator!=(
  const tlsf_heap_allocator<T, X> & a,
  const tlsf_heap_allocator<U, Y> & b) noexcept
{
  return a.memory_pool != b.memory_pool;
}

namespace rclcpp {

// Overload rclcpp::get_rcl_allocator for TLSF allocators.
template<typename T, size_t PoolSize>
rcl_allocator_t get_rcl_allocator(tlsf_heap_allocator<T, PoolSize> allocator)
{
  rcl_allocator_t rcl_allocator;
  rcl_allocator.allocate = [](size_t size, void *) {
      return tlsf_malloc(size);
    };
  rcl_allocator.deallocate = [](void * pointer, void *) {
      return tlsf_free(pointer);
    };
  rcl_allocator.reallocate = [](void * pointer, size_t size, void *) {
      return tlsf_realloc(pointer, size);
    };
  rcl_allocator.zero_allocate = [](size_t nmemb, size_t size, void *) {
      return tlsf_calloc(nmemb, size);
    };
  (void)allocator;  // unused
  return rcl_allocator;
}

}  // namespace rclcpp

#endif  // TLSF_CPP__TLSF_HPP_
