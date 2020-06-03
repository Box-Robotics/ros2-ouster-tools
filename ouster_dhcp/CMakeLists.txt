cmake_minimum_required(VERSION 3.5)
project(ouster_dhcp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

##############
## Install  ##
##############
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/dhcp-server
  DESTINATION lib/${PROJECT_NAME}/
  )

##############
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_package()
