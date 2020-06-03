cmake_minimum_required(VERSION 3.5)
project(ouster_h5)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(h5_bridge REQUIRED)
find_package(h5b_sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(H5_NODE_EXE h5_node)

#############
##  Build  ##
#############

#-----------------------
# exe: ${H5_NODE_EXE}
#-----------------------
add_executable(${H5_NODE_EXE} src/bin/h5_node.cpp)
target_compile_options(${H5_NODE_EXE}
  PRIVATE -Wall -Wextra
  )
target_include_directories(${H5_NODE_EXE}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
ament_target_dependencies(${H5_NODE_EXE}
  rclcpp
  sensor_msgs
  h5_bridge
  h5b_sensor_msgs
  )

##############
## Install  ##
##############

install(
  TARGETS
    ${H5_NODE_EXE}
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY etc
  DESTINATION share/${PROJECT_NAME}/
  )

#############
##  Test   ##
#############
if(BUILD_TESTING)
  # Add tests here
endif(BUILD_TESTING)


##############
ament_export_dependencies(ament_cmake)
ament_package()
