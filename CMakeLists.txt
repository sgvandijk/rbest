cmake_minimum_required(VERSION 2.10)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)

find_package(Eigen3 REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CTest)

set(COMMON_COMPILE_FLAGS "-std=c++14 -Wall -Wno-int-in-bool-context")

set(UNIT_TESTS
  test/TestSystemModel.cc
  test/TestObservationModel.cc
  test/TestKalmanFilter.cc
  )

include_directories(${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory("external/googletest")
  include_directories("external/googletest/include")
  add_executable(testrbest ${UNIT_TESTS})
  target_link_libraries(testrbest PRIVATE gtest gtest_main)
  set_target_properties(testrbest PROPERTIES COMPILE_FLAGS ${COMMON_COMPILE_FLAGS})
  add_test(rbest testrbest)
endif()
