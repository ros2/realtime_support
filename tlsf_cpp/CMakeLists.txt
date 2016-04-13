cmake_minimum_required(VERSION 2.8.3)

project(tlsf_cpp)
if(WIN32 OR APPLE)
  message(STATUS "tlsf allocator does not target Windows or OSX, skipping...")
  return()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tlsf REQUIRED)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # get the rmw implementations ahead of time
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(middleware_implementations)
  foreach(middleware_impl ${middleware_implementations})
    find_package("${middleware_impl}" REQUIRED)
  endforeach()

  # There is only one target
  set(target test_tlsf)

  # get typesupport of rmw implementation to include / link against the corresponding interfaces

  macro(add_gtest)
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")

    ament_add_gtest(${target}${target_suffix} "test/test_tlsf.cpp" TIMEOUT 15)
    if(TARGET ${target}${target_suffix})
      target_link_libraries(${target}${target_suffix}
        ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
        ${_AMENT_EXPORT_LIBRARY_TARGETS})
      target_compile_definitions(${target}${target_suffix}
        PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
      add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
      ament_target_dependencies(${target}${target_suffix}
        "rclcpp${target_suffix}" "std_msgs" "tlsf_cpp" "tlsf")
    endif()
  endmacro()

  call_for_each_rmw_implementation(add_gtest)
endif()

include_directories(include)

macro(example_target)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  add_executable(allocator_example${target_suffix}
    example/allocator_example.cpp)
  ament_target_dependencies(allocator_example${target_suffix}
    "rclcpp${target_suffx}"
    "std_msgs"
    "tlsf")
  install(TARGETS allocator_example${target_suffix}
    DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(example_target GENERATE_DEFAULT)

ament_export_include_directories(include)
ament_export_dependencies("tlsf")

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
