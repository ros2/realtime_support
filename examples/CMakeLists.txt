cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rttest_examples)
#set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(rttest REQUIRED)

link_directories(${rttest_LIBRARY_DIR})
include_directories(${rttest_INCLUDE_DIRS})


add_executable(example_loop example_loop.c)
target_link_libraries(example_loop ${rttest_LIBRARIES})
