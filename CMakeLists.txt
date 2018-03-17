cmake_minimum_required(VERSION 2.8.3)
project(vision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  OpenCV REQUIRED
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(DIRECTORY msg FILES
  VisionMessage.msg)
generate_messages(DEPENDENCIES std_msgs)

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
  src/vision.cpp
)
target_link_libraries(vision_node ${catkin_LIBRARIES})
