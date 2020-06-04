cmake_minimum_required(VERSION 3.5)
project(ouster_tools)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

set(DRIVER_COMPONENT_MGR_EXE driver_component_manager)
set(PERF_NODE_EXE perf_node)

# ament index resource
set(node_plugins "")

#############
##  Build  ##
#############

#-----------------------
# libouster_tools_perf_component.so
#-----------------------
add_library(${PROJECT_NAME}_perf_component SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/perf/perf_node.cpp
  )

target_include_directories(${PROJECT_NAME}_perf_component
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

target_compile_options(${PROJECT_NAME}_perf_component
  PRIVATE -Wall -Wextra
  )

ament_target_dependencies(${PROJECT_NAME}_perf_component
  rclcpp
  rclcpp_components
  sensor_msgs
  )

rclcpp_components_register_nodes(
  ${PROJECT_NAME}_perf_component "ouster_tools::PerfNode"
  )
set(node_plugins
  "${node_plugins}ouster_tools::PerfNode;"
  )
set(node_plugins
  "${node_plugins}$<TARGET_FILE:${PROJECT_NAME}_perf_component>\n"
  )

#-----------------------
# exe: ${PERF_NODE_EXE}
#-----------------------
add_executable(${PERF_NODE_EXE} src/bin/perf_node.cpp)
target_compile_options(${PERF_NODE_EXE}
  PRIVATE -Wall -Wextra
  )
target_include_directories(${PERF_NODE_EXE}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
ament_target_dependencies(${PERF_NODE_EXE}
  rclcpp
  sensor_msgs
  )

#---------------------------------
# exe: ${DRIVER_COMPONENT_MGR_EXE}
#---------------------------------
add_executable(${DRIVER_COMPONENT_MGR_EXE}
  src/bin/driver_component_manager.cpp
  )
target_compile_options(${DRIVER_COMPONENT_MGR_EXE}
  PRIVATE -Wall -Wextra
  )
target_include_directories(${DRIVER_COMPONENT_MGR_EXE}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
ament_target_dependencies(${DRIVER_COMPONENT_MGR_EXE}
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  )

##############
## Install  ##
##############

install(
  TARGETS
    ${PROJECT_NAME}_perf_component

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  TARGETS
    ${PERF_NODE_EXE}
    ${DRIVER_COMPONENT_MGR_EXE}
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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
