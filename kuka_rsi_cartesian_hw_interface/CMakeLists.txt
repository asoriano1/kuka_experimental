cmake_minimum_required(VERSION 2.8.3)
project(kuka_rsi_cartesian_hw_interface)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  controller_manager
  hardware_interface
  joint_limits_interface
  realtime_tools
  roscpp
  std_msgs
  robotnik_trajectory_pad
  message_generation
  
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML REQUIRED)
add_service_files(
	FILES
	set_A1_A6.srv
)
generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    kuka_cartesian_hardware_interface
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
    std_msgs
    robotnik_msgs	
  DEPENDS
    TinyXML
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(kuka_cartesian_hardware_interface
  src/kuka_cartesian_hardware_interface.cpp
)

target_link_libraries(kuka_cartesian_hardware_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

add_executable(kuka_cartesian_hardware_interface_node
  src/kuka_cartesian_hardware_interface_node.cpp
)

target_link_libraries(kuka_cartesian_hardware_interface_node
  kuka_cartesian_hardware_interface
)

install(
  TARGETS kuka_cartesian_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  TARGETS kuka_cartesian_hardware_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
