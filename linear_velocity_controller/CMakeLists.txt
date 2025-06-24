cmake_minimum_required(VERSION 3.8)
project(linear_velocity_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(linear_velocity_controller_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ruckig REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/linear_velocity_controller.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
  PUBLIC ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME}
  PUBLIC c_std_99 # Require C99
  PUBLIC cxx_std_17 # Require C++17
)
target_link_libraries(${PROJECT_NAME}
  ruckig::ruckig
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  franka_semantic_components
  linear_velocity_controller_interfaces
  ruckig
)

pluginlib_export_plugin_description_file(
  controller_interface linear_velocity_controller.xml
)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Test to see if controllers can be loaded
  ament_add_gmock(${PROJECT_NAME}_test_load
    test/test_load_linear_velocity_controller.cpp
  )
  target_include_directories(${PROJECT_NAME}_test_load
    PRIVATE include
  )
  ament_target_dependencies(${PROJECT_NAME}_test_load
    controller_manager
    ros2_control_test_assets
  )

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Also install the include folder contents and the generated library
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
  ruckig::ruckig
)
ament_package()
