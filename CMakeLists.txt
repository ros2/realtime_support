cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rttest)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(INSTALL_LIB_DIR lib CACHE PATH "install dir for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "install dir for executables")
set(INSTALL_INCLUDE_DIR include/rttest CACHE PATH "install dir for headers")

set(DEF_INSTALL_CMAKE_DIR lib/cmake/rttest)
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "install dir for cmake files")

add_library(rttest STATIC ${PROJECT_SOURCE_DIR}/src/rttest.cpp)

include_directories(rttest ${PROJECT_SOURCE_DIR}/include/rttest)

install(TARGETS rttest
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)

install(FILES ${PROJECT_SOURCE_DIR}/include/rttest/rttest.h ${PROJECT_SOURCE_DIR}/include/rttest/utils.h DESTINATION ${INSTALL_INCLUDE_DIR})

install(FILES cmake/rttestConfig.cmake DESTINATION ${INSTALL_CMAKE_DIR})
