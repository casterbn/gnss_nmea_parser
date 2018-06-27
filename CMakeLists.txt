cmake_minimum_required(VERSION 2.8.3)
project(gnss_parse)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GPGGA.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

###########
## Build ##
###########

include_directories(
  include/gnss_parse
  ${catkin_INCLUDE_DIRS}
)

list(APPEND SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/global.cpp
#     ${PROJECT_SOURCE_DIR}/src/gnss_handler.cpp
)

add_library(gnss_parse SHARED ${SOURCE_FILES})
add_dependencies(gnss_parse ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(gnss_parse ${catkin_LIBRARIES})

add_executable(gnss_parse_node src/gnss_parse_node.cpp)
target_link_libraries(gnss_parse_node gnss_parse)
