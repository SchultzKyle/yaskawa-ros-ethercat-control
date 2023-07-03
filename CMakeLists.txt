# cmake_minimum_required(VERSION 3.0.2)
cmake_minimum_required(VERSION 2.8.3)
project(ethercat_master)

# C++11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  soem
  roscpp
  std_msgs
  genmsg
  control
  std_msgs
  message_generation
)

## Boost
find_package(Boost REQUIRED COMPONENTS
  program_options
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

add_message_files(
  FILES
  MotorCommands.msg
  )

add_executable(run src/run.cpp)

add_dependencies(run ${catkin_EXPORTED_TARGETS})

target_link_libraries(run ${catkin_LIBRARIES} ${Boost_LIBRARIES})

message(STATUS ${CMAKE_MODULE_PATH})
