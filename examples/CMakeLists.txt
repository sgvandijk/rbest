cmake_minimum_required(VERSION 3.5)

find_package(SDL2 REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS})

add_executable(flyer flyer.cc)

set_target_properties(flyer PROPERTIES COMPILE_FLAGS ${COMMON_COMPILE_FLAGS})
target_link_libraries(flyer ${SDL2_LIBRARIES})
