cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rttest)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(INSTALL_LIB_DIR lib CACHE PATH "install dir for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "install dir for executables")
set(INSTALL_INCLUDE_DIR include/rttest CACHE PATH "install dir for headers")

set(DEF_INSTALL_CMAKE_DIR lib/cmake/rttest)
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "install dir for cmake files")

add_library(rttest ${PROJECT_SOURCE_DIR}/src/rttest.cpp)

include_directories(rttest ${PROJECT_SOURCE_DIR}/include/rttest)

install(TARGETS rttest
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)

install(FILES ${PROJECT_SOURCE_DIR}/include/rttest/rttest.h ${PROJECT_SOURCE_DIR}/include/rttest/utils.h DESTINATION ${INSTALL_INCLUDE_DIR})

#install(FILES cmake/rttestConfig.cmake DESTINATION ${INSTALL_CMAKE_DIR})

export(PACKAGE ${PROJECT_NAME})

#set(rttest_INCLUDE_DIR ${CMAKE_SOURCE_DIR})
#set(rttest_LIB_DIR ${CMAKE_BINARY_DIR}/lib)
#set(rttest_CMAKE_DIR ${CMAKE_BINARY_DIR})
#configure_file(${CMAKE_PROJECT_NAME}Config.cmake.in
#  ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake @ONLY)

set(rttest_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(rttest_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(rttest_CMAKE_DIR
  ${CMAKE_INSTALL_PREFIX}/share/${CMAKE_PROJECT_NAME}/CMake)
configure_file(${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_BINARY_DIR}/InstallFiles/${CMAKE_PROJECT_NAME}Config.cmake
  @ONLY)
install(FILES
  ${CMAKE_BINARY_DIR}/InstallFiles/${CMAKE_PROJECT_NAME}Config.cmake
  DESTINATION ${rttest_CMAKE_DIR})
