cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rttest_examples)

find_package(rttest REQUIRED)

message("library path: ${rttest_LIBRARY_DIR}")
message("libraries: ${rttest_LIBRARIES}")
message("headers path: ${rttest_INCLUDE_DIRS}")

include_directories(${rttest_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11")

add_executable(example_loop example_loop.c)
target_link_libraries(example_loop ${rttest_LIBRARIES} m stdc++)
