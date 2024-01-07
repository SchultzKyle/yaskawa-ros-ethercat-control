# cmake_minimum_required(VERSION 3.0.2)
cmake_minimum_required(VERSION 2.8.3)
project(yaskawa_ethercat)

# C++11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  soem
  roscpp
  std_msgs
  genmsg
  std_msgs
  message_generation
)

## Boost
find_package(Boost REQUIRED COMPONENTS
  program_options
)

add_message_files(
  FILES
  MotorCommands.msg
  )
  
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
)

include_directories(
# include
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(run src/run.cpp)

add_dependencies(run ${catkin_EXPORTED_TARGETS})

target_link_libraries(run ${catkin_LIBRARIES} ${Boost_LIBRARIES})

message(STATUS ${CMAKE_MODULE_PATH})
