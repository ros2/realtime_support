cmake_minimum_required(VERSION 3.20)

project(rttest)

if(WIN32 OR APPLE OR ANDROID OR BSD)
  message(STATUS "rttest is only supported on Linux, skipping...")
  return()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)

# ament build if it was found
if(${ament_cmake_FOUND})
  find_package(ament_cmake_ros_core REQUIRED)
  find_package(Threads REQUIRED)

  add_library(rttest SHARED src/rttest.cpp)
  # ament_ros_defaults provides the C++20 (cxx_std_20) compile feature; linking
  # it PUBLIC propagates the standard to consumers, including the test targets.
  target_link_libraries(rttest
    PUBLIC
      ament_cmake_ros_core::ament_ros_defaults
    PRIVATE
      m stdc++ Threads::Threads)
  target_include_directories(rttest PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

  ament_export_targets(export_rttest)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(
      gtest_rttest_api
      "test/test_api.cpp"
      TIMEOUT 15
    )
    target_link_libraries(gtest_rttest_api rttest)

    ament_add_gtest(
      gtest_math_utils
      "test/test_math_util.cpp"
      TIMEOUT 15
    )
    target_link_libraries(gtest_math_utils rttest)
  endif()

  ament_package()

  install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )

  install(
    PROGRAMS scripts/rttest_plot
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    TARGETS rttest
    EXPORT export_rttest
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )

else()
  message(STATUS "ament_cmake_auto not found, executing pure CMake build")
  # pure cmake build
  set(INSTALL_LIB_DIR lib CACHE PATH "install dir for libraries")
  set(INSTALL_BIN_DIR bin CACHE PATH "install dir for executables")
  set(INSTALL_INCLUDE_DIR include CACHE PATH "install dir for headers")

  set(DEF_INSTALL_CMAKE_DIR lib/cmake/rttest)
  set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "install dir for cmake files")

  find_package(Threads REQUIRED)
  add_library(rttest ${PROJECT_SOURCE_DIR}/src/rttest.cpp)
  # No ament_cmake_ros_core available in the pure CMake build; set C++20 directly.
  target_compile_features(rttest PUBLIC cxx_std_20)
  target_link_libraries(rttest m stdc++ ${CMAKE_THREAD_LIBS_INIT})

  include_directories(rttest ${PROJECT_SOURCE_DIR}/include)

  install(TARGETS rttest
    LIBRARY DESTINATION ${INSTALL_LIB_DIR}
    ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
  )

  install(
    DIRECTORY include/
    DESTINATION ${INSTALL_INCLUDE_DIR})

  export(PACKAGE ${PROJECT_NAME})

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

endif()
