cmake_minimum_required(VERSION 3.5)

project(tlsf_cpp)
if(WIN32 OR APPLE OR ANDROID)
  message(STATUS "tlsf allocator does not target Windows, OSX or Android, skipping...")
  return()
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tlsf REQUIRED)

include_directories(include)

add_executable(tlsf_allocator_example
  example/allocator_example.cpp)
ament_target_dependencies(tlsf_allocator_example
  "rclcpp"
  "std_msgs"
  "tlsf")
install(TARGETS tlsf_allocator_example
  DESTINATION bin)

ament_export_include_directories(include)
ament_export_dependencies("tlsf")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # There is only one target
  set(target test_tlsf)

  # get typesupport of rmw implementation to include / link against the corresponding interfaces

  macro(add_gtest)
    ament_add_gtest(${target}${target_suffix} "test/test_tlsf.cpp" TIMEOUT 15
      ENV
      RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
      RMW_IMPLEMENTATION=${rmw_implementation})
    if(TARGET ${target}${target_suffix})
      target_compile_definitions(${target}${target_suffix}
        PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
      ament_target_dependencies(${target}${target_suffix}
        "rclcpp" "std_msgs" "tlsf")
    endif()
  endmacro()

  call_for_each_rmw_implementation(add_gtest)
endif()

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
