cmake_minimum_required(VERSION 2.8.3)
project(distributed_randomized_sequential_partitions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_message_files(
  FILES
  SimpleDistributedPlannerMsg.msg
  DistributedPlannerSignature.msg
  DistributedPlannerStatistics.msg
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
)

add_compile_options(-std=c++17)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(
  ${catkin_DEFINITIONS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(simple_distributed_planning_node src/simple_distributed_planning_node.cpp)
add_dependencies(simple_distributed_planning_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(simple_distributed_planning_node
  ${catkin_LIBRARIES}
)
