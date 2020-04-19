cmake_minimum_required(VERSION 3.5)
project(ouster_tools)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(PERF_NODE_EXE perf_node)
set(PMC_NODE_EXE pmc_node)
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
set(PTP_SRC_DIR ${THIRD_PARTY_DIR}/linuxptp)

if(EXISTS ${PTP_SRC_DIR})
  execute_process(
    COMMAND ${PTP_SRC_DIR}/incdefs.sh
    WORKING_DIRECTORY ${PTP_SRC_DIR}
    OUTPUT_VARIABLE PTP_INCDEFS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  message(STATUS "PTP_INCDEFS: ${PTP_INCDEFS}")
  string(REPLACE " " ";" PTP_INCDEFS_LIST ${PTP_INCDEFS})

  execute_process(
    COMMAND ${PTP_SRC_DIR}/version.sh
    WORKING_DIRECTORY ${PTP_SRC_DIR}
    OUTPUT_VARIABLE PTP_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  message(STATUS "PTP_VERSION: ${PTP_VERSION}")
endif()

#############
##  Build  ##
#############

#-----------------------
# liblinuxptp_pmc.so
#-----------------------

add_library(linuxptp_pmc SHARED
  ${PTP_SRC_DIR}/config.c
  ${PTP_SRC_DIR}/hash.c
  ${PTP_SRC_DIR}/interface.c
  ${PTP_SRC_DIR}/msg.c
  ${PTP_SRC_DIR}/phc.c
  ${PTP_SRC_DIR}/pmc_common.c
  ${PTP_SRC_DIR}/print.c
  ${PTP_SRC_DIR}/sk.c
  ${PTP_SRC_DIR}/tlv.c
  ${PTP_SRC_DIR}/raw.c
  ${PTP_SRC_DIR}/transport.c
  ${PTP_SRC_DIR}/udp.c
  ${PTP_SRC_DIR}/udp6.c
  ${PTP_SRC_DIR}/uds.c
  ${PTP_SRC_DIR}/util.c
  ${PTP_SRC_DIR}/version.c
  )

target_include_directories(linuxptp_pmc
  PRIVATE ${PTP_SRC_DIR}
  )

target_compile_definitions(linuxptp_pmc
  PRIVATE
  VER=${PTP_VERSION}
  )

foreach(PTP_INCDEF in ${PTP_INCDEFS_LIST})
  target_compile_definitions(linuxptp_pmc
    PRIVATE ${PTP_INCDEF}
    )
endforeach()

target_compile_options(linuxptp_pmc
  PRIVATE -Wall -Wno-format-truncation -Wno-address-of-packed-member
  )

#-----------------------
# libouster_tools_ptp.so
#-----------------------

add_library(${PROJECT_NAME}_ptp SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/ptp/err.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/ptp/pmc.cpp
  )

target_include_directories(${PROJECT_NAME}_ptp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/ptp
    ${THIRD_PARTY_DIR}
  )

target_compile_options(${PROJECT_NAME}_ptp
  PRIVATE -Wall -Wextra -Wno-address-of-packed-member
  )

target_link_libraries(${PROJECT_NAME}_ptp
  linuxptp_pmc
  )

#-----------------------
# exe: ${PMC_NODE_EXE}
#-----------------------
add_executable(${PMC_NODE_EXE} src/bin/pmc_node.cpp)
target_compile_options(${PMC_NODE_EXE}
  PRIVATE -Wall -Wextra
  )
target_include_directories(${PMC_NODE_EXE}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
target_link_libraries(${PMC_NODE_EXE}
  ${PROJECT_NAME}_ptp
  )
ament_target_dependencies(${PMC_NODE_EXE}
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  std_msgs
  )

#-----------------------
# exe: ${PERF_NODE_EXE}
#-----------------------
add_executable(${PERF_NODE_EXE} src/bin/perf_node.cpp)
target_compile_options(${PERF_NODE_EXE}
  PRIVATE -Wall -Wextra
  )
ament_target_dependencies(${PERF_NODE_EXE}
  rclcpp
  sensor_msgs
  )


##############
## Install  ##
##############
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/dhcp-server
    scripts/ptp-dump
    scripts/sys-time
  DESTINATION lib/${PROJECT_NAME}/
  )

install(
  TARGETS
    linuxptp_pmc
    ${PROJECT_NAME}_ptp

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  TARGETS
    ${PMC_NODE_EXE}
    ${PERF_NODE_EXE}
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
ament_export_dependencies(ament_cmake_python)
ament_package()
