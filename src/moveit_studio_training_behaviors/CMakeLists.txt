cmake_minimum_required(VERSION 3.16.3)
project(moveit_studio_training_behaviors)

# Set MoveIt's typical CMake settings, including C++ standard version
find_package(moveit_common REQUIRED)
moveit_package()

find_package(apriltag_ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_studio_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml-cpp REQUIRED)

set(
  THIS_PACKAGE_INCLUDE_DEPENDS
  apriltag_ros_msgs
  geometry_msgs
  moveit_studio_behavior_interface
  pluginlib
  sensor_msgs
  tf2_eigen
  yaml-cpp
)

# AprilTag Behavior
add_library(
  moveit_studio_training_behaviors SHARED
  src/get_apriltag_detection_pose.cpp
  src/setup_mtc_pick_from_pose.cpp
  src/offset_pose.cpp
  src/register_behaviors.cpp
)
target_include_directories(
  moveit_studio_training_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(moveit_studio_training_behaviors ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_studio_training_behaviors ${YAML_CPP_LIBRARIES})

# Install Libraries
install(
  TARGETS moveit_studio_training_behaviors
  EXPORT moveit_studio_training_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  add_subdirectory(test)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the behavior plugins defined in this package so they are available to plugin loaders that load the
# behavior base class library from the moveit_studio_behavior package.
pluginlib_export_plugin_description_file(moveit_studio_behavior_interface moveit_studio_training_behaviors_plugin_description.xml)

ament_export_targets(moveit_studio_training_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
