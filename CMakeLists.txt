cmake_minimum_required(VERSION 3.8)
project(diff_canopen_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(canopen_ros2_control REQUIRED)

add_library(
  diff_canopen_system
  SHARED
  src/DiffCanopenSystem.cpp
)
target_include_directories(
  diff_canopen_system
  PUBLIC
  include
)
ament_target_dependencies(
  diff_canopen_system
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  canopen_ros2_control
)
# prevent pluginlib from using boost
target_compile_definitions(diff_canopen_system PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface diff_canopen_system.xml)

install(
  TARGETS
  diff_canopen_system
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  diff_canopen_system
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
