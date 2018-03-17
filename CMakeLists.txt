cmake_minimum_required(VERSION 2.8.3)
project(vision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

find_package(OpenCV REQUIRED)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
)
add_executable(vision_node
  src/main.cpp
)
target_link_libraries(vision_node ${catkin_LIBRARIES})
