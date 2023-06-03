cmake_minimum_required(VERSION 3.5)
project(starter_files)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
