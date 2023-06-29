cmake_minimum_required(VERSION 3.16.3)
project(offset_pose)

# Set MoveIt's typical CMake settings, including C++ standard version
find_package(moveit_common REQUIRED)
moveit_package()

find_package(moveit_studio_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(
  THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_studio_behavior_interface
  pluginlib
  geometry_msgs
  tf2_eigen
)

# AprilTag Behavior
add_library(
  offset_pose SHARED
  src/offset_pose.cpp
  src/register_behaviors.cpp
)
target_include_directories(
  offset_pose
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(offset_pose ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS offset_pose
  EXPORT offset_poseTargets
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
pluginlib_export_plugin_description_file(moveit_studio_behavior_interface offset_pose_plugin_description.xml)

ament_export_targets(offset_poseTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
