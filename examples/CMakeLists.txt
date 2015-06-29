cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rttest_examples)

set(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/rttest:${CMAKE_MODULE_PATH})

find_package(rttest REQUIRED)

message("module path: ${CMAKE_MODULE_PATH}")
message("library path: ${rttest_LIBRARIES}")
message("headers path: ${rttest_INCLUDE_DIRS}")

include_directories(${rttest_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11")

add_executable(example_loop example_loop.c)
target_link_libraries(example_loop ${rttest_LIBRARIES} m stdc++)
