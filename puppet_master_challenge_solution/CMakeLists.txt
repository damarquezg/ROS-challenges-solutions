cmake_minimum_required(VERSION 2.8.3)
project(puppet_master_challenge_solution)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  turtlesim
  puppet_master_challenge
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES puppet_master_challenge_solution
  CATKIN_DEPENDS roscpp rospy std_msgs puppet_master_challenge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(HEADERS
  include/${PROJECT_NAME}/puppet_master_controller.h
)

set(SOURCES
  src/puppet_master_controller.cpp
)

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

add_executable(
  puppet_master_controller src/puppet_master_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${boost_LIBRARIES})
target_link_libraries(puppet_master_controller ${PROJECT_NAME})

install(TARGETS puppet_master_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/run_server.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
