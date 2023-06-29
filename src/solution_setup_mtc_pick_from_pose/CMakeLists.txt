cmake_minimum_required(VERSION 3.16.3)
project(setup_mtc_pick_from_pose)

# Set MoveIt's typical CMake settings, including C++ standard version
find_package(moveit_common REQUIRED)
moveit_package()

find_package(moveit_studio_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml-cpp REQUIRED)

set(
  THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_studio_behavior_interface
  pluginlib
  sensor_msgs
  tf2_eigen
  yaml-cpp
)

# AprilTag Behavior
add_library(
  setup_mtc_pick_from_pose SHARED
  src/setup_mtc_pick_from_pose.cpp
  src/register_behaviors.cpp
)
target_include_directories(
  setup_mtc_pick_from_pose
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(setup_mtc_pick_from_pose ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(setup_mtc_pick_from_pose ${YAML_CPP_LIBRARIES})

# Install Libraries
install(
  TARGETS setup_mtc_pick_from_pose
  EXPORT setup_mtc_pick_from_poseTargets
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
pluginlib_export_plugin_description_file(moveit_studio_behavior_interface setup_mtc_pick_from_pose_plugin_description.xml)

ament_export_targets(setup_mtc_pick_from_poseTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
