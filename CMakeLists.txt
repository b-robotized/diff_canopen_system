cmake_minimum_required(VERSION 3.16)
project(rpm_powertrain_driver LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}_diff_canopen_system SHARED
  src/DiffCanopenSystem.cpp
)
target_compile_features(${PROJECT_NAME}_diff_canopen_system PUBLIC c_std_99 cxx_std_17)
target_include_directories(${PROJECT_NAME}_diff_canopen_system PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/rpm_powertrain_driver>
)
ament_target_dependencies(${PROJECT_NAME}_diff_canopen_system  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(
  hardware_interface ${PROJECT_NAME}.xml)

add_executable(fake_canopen_inverter_node src/fake_canopen/fake_inverter.cpp)
target_compile_features(fake_canopen_inverter_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(fake_canopen_inverter_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/rpm_powertrain_driver>
)
ament_target_dependencies(fake_canopen_inverter_node  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

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

  ament_add_gmock(test_DiffCanopenSystem test/test_DiffCanopenSystem.cpp)
  target_include_directories(test_DiffCanopenSystem PRIVATE include)
  ament_target_dependencies(
    test_DiffCanopenSystem
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

## INSTALL ##

cogen_dcf(fake_rpm_system)

install(DIRECTORY launch ros2_control_config urdf
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY include/
  DESTINATION include/rpm_powertrain_driver
)
install(
  TARGETS ${PROJECT_NAME}_diff_canopen_system
  EXPORT export_rpm_powertrain_driver
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(
  TARGETS fake_canopen_inverter_node
  RUNTIME DESTINATION lib/rpm_powertrain_driver
)

ament_export_targets(export_rpm_powertrain_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
