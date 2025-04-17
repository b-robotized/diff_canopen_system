cmake_minimum_required(VERSION 3.16)
project(diff_canopen_system LANGUAGES C CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  canopen_fake_slaves
  canopen_ros2_control
  hardware_interface
  lely_core_libraries
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#  Diff System
add_library(diff_canopen_system SHARED
  src/diff_canopen_system.cpp
  src/diff_canopen_system_multi_rpdo.cpp
)
target_compile_features(diff_canopen_system PUBLIC c_std_99 cxx_std_17)
target_include_directories(diff_canopen_system PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diff_canopen_system>
)
ament_target_dependencies(diff_canopen_system  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ros2_control plugins
pluginlib_export_plugin_description_file(
  hardware_interface ${PROJECT_NAME}.xml)

# Fake Inverter Node for testing
add_executable(fake_canopen_controller_node src/fake_canopen/fake_motor_controller.cpp)
target_compile_features(fake_canopen_controller_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(fake_canopen_controller_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diff_canopen_system>
)
ament_target_dependencies(fake_canopen_controller_node  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # ament_add_gmock(test_DiffCanopenSystem test/test_DiffCanopenSystem.cpp)
  # target_include_directories(test_DiffCanopenSystem PRIVATE include)
  # ament_target_dependencies(
  #   test_DiffCanopenSystem
  #   hardware_interface
  #   pluginlib
  #   ros2_control_test_assets
  # )
endif()

## INSTALL ##

cogen_dcf(fake_diff_drive_system)
# cogen_dcf(test_diff_canopen_system)

install(DIRECTORY launch ros2_control_config urdf
  DESTINATION share/diff_canopen_system/
)
install(
  DIRECTORY include/
  DESTINATION include/diff_canopen_system
)
install(
  TARGETS diff_canopen_system
  EXPORT export_diff_canopen_system
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(
  TARGETS fake_canopen_controller_node
  RUNTIME DESTINATION lib/diff_canopen_system
)


ament_export_targets(export_diff_canopen_system HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
