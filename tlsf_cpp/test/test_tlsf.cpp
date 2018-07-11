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

#include <memory>
#include <string>
#include <vector>
#include "gtest/gtest.h"

#ifdef __GNUC__
#include <cxxabi.h>
#include <execinfo.h>
#include <malloc.h>
#endif

#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/u_int32.hpp"
#include "tlsf_cpp/tlsf.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif


// TODO(jacquelinekay) improve this ignore rule (dogfooding or no allocations)
static const size_t num_rmw_tokens = 7;
static const char * rmw_tokens[num_rmw_tokens] = {
  "librmw", "dds", "DDS", "dcps", "DCPS", "fastrtps", "opensplice"
};

static const size_t iterations = 1;

static bool verbose = false;
static bool ignore_middleware_tokens = true;
static bool test_init = false;
static bool fail = false;

inline bool check_stacktrace(const char ** tokens, size_t num_tokens, size_t max_frames = 15);

/// Declare a function pointer into which we will store the default malloc.
static void * (* prev_malloc_hook)(size_t, const void *);

// Use pragma to ignore a warning for using __malloc_hook, which is deprecated (but still awesome).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

static void * testing_malloc(size_t size, const void * caller)
{
  (void)caller;
  // Set the malloc implementation to the default malloc hook so that we can call it implicitly
  // to initialize a string, otherwise this function will loop infinitely.
  __malloc_hook = prev_malloc_hook;

  if (test_init) {
    fail |= !check_stacktrace(rmw_tokens, num_rmw_tokens);
  }

  // Execute the requested malloc.
  void * mem = std::malloc(size);
  // Set the malloc hook back to this function, so that we can intercept future mallocs.
  __malloc_hook = testing_malloc;
  return mem;
}

/// Function to be called when the malloc hook is initialized.
void init_malloc_hook()
{
  // Store the default malloc.
  prev_malloc_hook = __malloc_hook;
  // Set our custom malloc to the malloc hook.
  __malloc_hook = testing_malloc;
}
#pragma GCC diagnostic pop


/// Set the hook for malloc initialize so that init_malloc_hook gets called.
void(*volatile __malloc_initialize_hook)(void) = init_malloc_hook;

/** Check a demangled stack backtrace of the caller function for the given tokens.
 ** Adapted from: https://panthema.net/2008/0901-stacktrace-demangled
 **/
bool check_stacktrace(const char ** tokens, size_t num_tokens, size_t max_frames)
{
#ifdef __GNUC__
  bool match = false;

  // storage array for stack trace address data
  void * addrlist[max_frames + 1];

  // retrieve current stack addresses
  int addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void *));

  if (addrlen == 0) {
    fprintf(stderr, "WARNING: stack trace empty, possibly corrupt\n");
    return false;
  }

  // resolve addresses into strings containing "filename(function+address)",
  // this array must be free()-ed
  char ** symbollist = backtrace_symbols(addrlist, addrlen);

  // initialize string string which will be filled with the demangled function name
  // allocate string which will be filled with the demangled function name
  size_t funcnamesize = 256;
  char * funcname = static_cast<char *>(std::malloc(funcnamesize));


  if (verbose) {
    fprintf(stderr, ">>>> stack trace:\n");
  }

  // iterate over the returned symbol lines. skip the first, it is the
  // address of this function.
  for (int i = 1; i < addrlen; i++) {
    char * begin_name = 0, * begin_offset = 0, * end_offset = 0;

    // find parentheses and +address offset surrounding the mangled name:
    // ./module(function+0x15c) [0x8048a6d]
    for (char * p = symbollist[i]; *p; ++p) {
      if (*p == '(') {
        begin_name = p;
      } else if (*p == '+') {
        begin_offset = p;
      } else if (*p == ')' && begin_offset) {
        end_offset = p;
        break;
      }
    }

    if (begin_name && begin_offset && end_offset &&
      begin_name < begin_offset)
    {
      *begin_name++ = '\0';
      *begin_offset++ = '\0';
      *end_offset = '\0';

      int status;
      char * ret = abi::__cxa_demangle(begin_name, funcname, &funcnamesize, &status);
      if (status == 0) {
        funcname = ret;  // use possibly realloc()-ed string
        for (size_t j = 0; j < num_tokens; ++j) {
          if (strstr(symbollist[i],
            tokens[j]) != nullptr || strstr(funcname, tokens[j]) != nullptr)
          {
            match = true;
            break;
          }
        }
        if (verbose) {
          fprintf(stderr, "  %s : %s+%s\n", symbollist[i], funcname, begin_offset);
        }
      } else {
        // demangling failed. Output function name as a C function with
        // no arguments.
        for (size_t j = 0; j < num_tokens; j++) {
          if (strstr(symbollist[i],
            tokens[j]) != nullptr || strstr(begin_name, tokens[j]) != nullptr)
          {
            match = true;
            break;
          }
        }
        if (verbose) {
          fprintf(stderr, "  %s : %s()+%s\n", symbollist[i], begin_name, begin_offset);
        }
      }
    } else {
      // couldn't parse the line? print the whole line.
      for (size_t j = 0; j < num_tokens; j++) {
        if (strstr(symbollist[i], tokens[j]) != nullptr) {
          match = true;
          break;
        }
      }
      if (verbose) {
        fprintf(stderr, "  %s\n", symbollist[i]);
      }
    }
  }

  free(funcname);
  free(symbollist);
  if (!ignore_middleware_tokens) {
    return false;
  }
  return match;
#else
  return true;
#endif  // __GNUC__
}

void * operator new(std::size_t size)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  __malloc_hook = prev_malloc_hook;

  if (test_init) {
    // Check the stacktrace to see the call originated in librmw or a DDS implementation
    fail |= !check_stacktrace(rmw_tokens, num_rmw_tokens);
  }
  void * ptr = std::malloc(size);

  __malloc_hook = testing_malloc;
#pragma GCC diagnostic pop
  return ptr;
}

void operator delete(void * ptr) noexcept
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  __malloc_hook = prev_malloc_hook;

  if (ptr != nullptr) {
    if (test_init) {
      // Check the stacktrace to see the call originated in librmw or a DDS implementation
      fail |= !check_stacktrace(rmw_tokens, num_rmw_tokens);
    }

    std::free(ptr);
    ptr = nullptr;
  }
  __malloc_hook = testing_malloc;
#pragma GCC diagnostic pop
}

//  In C++14, (some) compilers emit a warning when the user has overridden
//  the unsized version of delete but not the sized version.
//  see http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2013/n3536.html
//  "The workaround is to define a sized version that simply calls the unsized
//  version."
void operator delete(void * ptr, size_t sz) noexcept
{
  (void)sz;  // unused parameter, since we're passing this to unsized delete
  operator delete(ptr);
}

template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

class CLASSNAME (AllocatorTest, RMW_IMPLEMENTATION) : public ::testing::Test
{
protected:
  std::string test_name_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;
  rclcpp::Publisher<
    std_msgs::msg::UInt32, TLSFAllocator<void>>::SharedPtr publisher_;
  rclcpp::message_memory_strategy::MessageMemoryStrategy<
    std_msgs::msg::UInt32, TLSFAllocator<void>>::SharedPtr msg_memory_strategy_;
  std::shared_ptr<TLSFAllocator<void>> alloc;

  bool intra_process_;

  using UInt32Allocator = TLSFAllocator<std_msgs::msg::UInt32>;
  using UInt32Deleter = rclcpp::allocator::Deleter<UInt32Allocator, std_msgs::msg::UInt32>;

  void initialize(bool intra_process, const std::string & name)
  {
    test_name_ = name;
    intra_process_ = intra_process;

    auto context = rclcpp::contexts::default_context::get_global_default_context();
    auto intra_process_manager_state =
      std::make_shared<rclcpp::intra_process_manager::IntraProcessManagerImpl<UInt32Allocator>>();
    context->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>(
      intra_process_manager_state);

    const std::vector<std::string> arguments = {};
    const std::vector<rclcpp::Parameter> initial_values = {};
    const bool use_global_arguments = true;
    node_ = rclcpp::Node::make_shared(
      name, "", context, arguments, initial_values, use_global_arguments, intra_process);
    alloc = std::make_shared<TLSFAllocator<void>>();
    msg_memory_strategy_ = std::make_shared<
      rclcpp::message_memory_strategy::MessageMemoryStrategy<
        std_msgs::msg::UInt32, TLSFAllocator<void>>>(alloc);
    publisher_ = node_->create_publisher<std_msgs::msg::UInt32>(name, 10, alloc);
    memory_strategy_ =
      std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>(alloc);

    rclcpp::executor::ExecutorArgs args;
    args.memory_strategy = memory_strategy_;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(args);

    executor_->add_node(node_);
  }

  CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION)() {
  }

  ~CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION)() {
  }
};

TEST_F(CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION), type_traits_test) {
  using UInt32TLSFAllocator = TLSFAllocator<std_msgs::msg::UInt32>;
  using UInt32TLSFDeleter = rclcpp::allocator::Deleter<UInt32TLSFAllocator, std_msgs::msg::UInt32>;

  auto cb_tlsf = [](std_msgs::msg::UInt32::UniquePtrWithDeleter<UInt32TLSFDeleter> msg) -> void
    {
      (void) msg;
    };
  static_assert(
    std::is_same<
      std_msgs::msg::UInt32,
      rclcpp::subscription_traits::has_message_type<decltype(cb_tlsf)>::type>::value,
    "tlsf unique ptr failed");

  using UInt32VoidAllocator = std::allocator<std_msgs::msg::UInt32>;
  using UInt32VoidDeleter = rclcpp::allocator::Deleter<UInt32VoidAllocator, std_msgs::msg::UInt32>;

  auto cb_void = [](std_msgs::msg::UInt32::UniquePtrWithDeleter<UInt32VoidDeleter> msg) -> void
    {
      (void) msg;
    };
  static_assert(
    std::is_same<
      std_msgs::msg::UInt32,
      rclcpp::subscription_traits::has_message_type<decltype(cb_void)>::type>::value,
    "void unique ptr failed");
}

TEST_F(CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION), allocator_shared_ptr) {
  initialize(false, "allocator_shared_ptr");
  size_t counter = 0;
  auto callback = [&counter](std_msgs::msg::UInt32::SharedPtr msg) -> void
    {
      EXPECT_EQ(counter, msg->data);
      counter++;
    };

  rclcpp::subscription_traits::has_message_type<decltype(callback)>::type a;
  auto subscriber = node_->create_subscription<std_msgs::msg::UInt32>(
    "allocator_shared_ptr", callback, rmw_qos_profile_default, nullptr, false, msg_memory_strategy_,
    alloc);
  // Create msg to be published
  auto msg = std::allocate_shared<std_msgs::msg::UInt32>(*alloc.get());

  rclcpp::sleep_for(std::chrono::milliseconds(1));
  // After test_initialization, global new should only be called from within TLSFAllocator.
  test_init = true;
  for (uint32_t i = 0; i < iterations; i++) {
    msg->data = i;
    publisher_->publish(msg);
    rclcpp::sleep_for(std::chrono::milliseconds(1));
    executor_->spin_some();
  }
  test_init = false;
  EXPECT_FALSE(fail);
  fail = false;
}

TEST_F(CLASSNAME(AllocatorTest, RMW_IMPLEMENTATION), allocator_unique_ptr) {
  initialize(true, "allocator_unique_ptr");
  size_t counter = 0;
  auto callback =
    [&counter](std::unique_ptr<std_msgs::msg::UInt32, UInt32Deleter> msg) -> void
    {
      EXPECT_EQ(counter, msg->data);
      counter++;
    };

  static_assert(
    std::is_same<
      std_msgs::msg::UInt32,
      rclcpp::subscription_traits::has_message_type<decltype(callback)>::type>::value,
    "passing a std::unique_ptr of test_msgs::msg::Empty has message type Empty");

  auto subscriber = node_->create_subscription<std_msgs::msg::UInt32>(
    "allocator_unique_ptr", callback, 10, nullptr, false, msg_memory_strategy_, alloc);

  TLSFAllocator<std_msgs::msg::UInt32> msg_alloc;

  // After test_initialization, global new should only be called from within TLSFAllocator.
  test_init = true;
  for (uint32_t i = 0; i < iterations; i++) {
    auto msg = std::unique_ptr<std_msgs::msg::UInt32, UInt32Deleter>(
      std::allocator_traits<UInt32Allocator>::allocate(msg_alloc, 1));
    msg->data = i;
    publisher_->publish(msg);
    rclcpp::sleep_for(std::chrono::milliseconds(1));
    executor_->spin_some();
  }
  test_init = false;
  EXPECT_FALSE(fail);
  fail = false;
}

void print_help()
{
  printf("--all-tokens: Do not ignore middleware tokens.\n");
  printf("--verbose: Report stack traces and allocation statistics.\n");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  // argc and argv are modified by InitGoogleTest
  std::vector<std::string> args(argv + 1, argv + argc);

  if (std::find(args.begin(), args.end(), "--help") != args.end()) {
    print_help();
    return 0;
  }
  verbose = std::find(args.begin(), args.end(), "--verbose") != args.end();
  ignore_middleware_tokens = std::find(args.begin(), args.end(), "--all-tokens") == args.end();

  return RUN_ALL_TESTS();
}
