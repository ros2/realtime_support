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

#include <list>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tlsf_cpp/tlsf.hpp"

template<typename T>
using TLSFAllocator = tlsf_heap_allocator<T>;

int main(int argc, char ** argv)
{
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node;

  std::list<std::string> keys = {"intra", "intraprocess", "intra-process", "intra_process"};
  bool intra_process = false;

  if (argc > 1) {
    for (auto & key : keys) {
      if (std::string(argv[1]) == key) {
        intra_process = true;
        break;
      }
    }
  }

  if (intra_process) {
    printf("Intra-process pipeline is ON.\n");
    auto context = rclcpp::contexts::default_context::get_global_default_context();
    auto ipm_state =
      std::make_shared<
      rclcpp::intra_process_manager::IntraProcessManagerImpl<TLSFAllocator<void>>>();
    // Constructs the intra-process manager with a custom allocator.
    context->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>(ipm_state);
    node = rclcpp::Node::make_shared("allocator_example", "", true);
  } else {
    printf("Intra-process pipeline is OFF.\n");
    node = rclcpp::Node::make_shared("allocator_example", "", false);
  }

  uint32_t counter = 0;
  auto callback = [&counter](std_msgs::msg::UInt32::SharedPtr msg) -> void
    {
      (void)msg;
      ++counter;
    };

  // Create a custom allocator and pass the allocator to the publisher and subscriber.
  auto alloc = std::make_shared<TLSFAllocator<void>>();
  auto publisher = node->create_publisher<std_msgs::msg::UInt32>("allocator_example", 10, alloc);
  auto msg_mem_strat = std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<
      std_msgs::msg::UInt32, TLSFAllocator<void>>>(alloc);
  auto subscriber = node->create_subscription<std_msgs::msg::UInt32>(
    "allocator_example", callback, 10, nullptr, false, msg_mem_strat, alloc);

  // Create a MemoryStrategy, which handles the allocations made by the Executor during the
  // execution path, and inject the MemoryStrategy into the Executor.
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>(alloc);

  rclcpp::executor::ExecutorArgs args;
  args.memory_strategy = memory_strategy;
  rclcpp::executors::SingleThreadedExecutor executor(args);

  // Add our node to the executor.
  executor.add_node(node);

  // Create a message with the custom allocator, so that when the Executor deallocates the
  // message on the execution path, it will use the custom deallocate.
  auto msg = std::allocate_shared<std_msgs::msg::UInt32>(*alloc.get());

  rclcpp::sleep_for(std::chrono::milliseconds(1));


  uint32_t i = 0;
  while (rclcpp::ok() && i < 100) {
    msg->data = i;
    ++i;
    publisher->publish(msg);
    rclcpp::sleep_for(std::chrono::milliseconds(1));
    executor.spin_some();
  }

  return 0;
}
