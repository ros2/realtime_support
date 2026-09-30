cmake_minimum_required(VERSION 3.20)

project(tlsf_cpp)
if(WIN32 OR APPLE OR ANDROID)
  message(STATUS "tlsf allocator does not target Windows, OSX or Android, skipping...")
  return()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_ros_core REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tlsf REQUIRED)


add_library(tlsf_cpp INTERFACE)
target_include_directories(tlsf_cpp INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(tlsf_cpp INTERFACE
  ament_cmake_ros_core::ament_ros_defaults
  rcl::rcl
  tlsf::tlsf
)

add_executable(tlsf_allocator_example
  example/allocator_example.cpp)
target_link_libraries(tlsf_allocator_example PRIVATE
  rclcpp::rclcpp
  std_msgs::std_msgs
  tlsf_cpp
)

install(TARGETS
  tlsf_allocator_example
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_tlsf_cpp)

ament_export_dependencies(ament_cmake_ros_core)
ament_export_dependencies("rcl" "tlsf")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)
  find_package(rcpputils REQUIRED)

  ament_add_gtest_executable(test_tlsf test/test_tlsf.cpp)
  target_link_libraries(test_tlsf
    rclcpp::rclcpp
    rcpputils::rcpputils
    std_msgs::std_msgs
    tlsf_cpp
  )

  function(add_gtest)
    ament_add_ros_isolated_gtest_test(test_tlsf
      TEST_NAME test_tlsf${target_suffix}
      TIMEOUT 15
      ENV
        RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
        RMW_IMPLEMENTATION=${rmw_implementation})
  endfunction()
  call_for_each_rmw_implementation(add_gtest)
endif()

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS tlsf_cpp EXPORT export_tlsf_cpp)

ament_package()
