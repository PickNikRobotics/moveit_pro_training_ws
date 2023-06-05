cmake_minimum_required(VERSION 3.16.3)
project(apriltag_ros_msgs)

# Disable Wredundant-decls warnings since rosidl generates redundant function declarations
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-redundant-decls")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/Detection.msg"
)

set(srv_files
  "srv/GetAprilTagDetections.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
