cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rttest)

set(CMAKE_CXX_FLAGS "-std=c++11")

add_library(rttest src/rttest.cpp)

include_directories(rttest include/rttest)
