cmake_minimum_required(VERSION 3.8)
project(print_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gcode_to_path REQUIRED)
find_package(message_filters REQUIRED)
add_executable(PrintController src/PrintController.cpp)
add_executable(PrintVisualization src/PrintVisualization.cpp)
add_executable(PayloadPositionPublisher src/PayloadPositionPublisher.cpp)
ament_target_dependencies(PrintController rclcpp geometry_msgs visualization_msgs gcode_to_path)
ament_target_dependencies(PrintVisualization rclcpp geometry_msgs visualization_msgs gcode_to_path)
ament_target_dependencies(PayloadPositionPublisher rclcpp nav_msgs message_filters)

target_include_directories(PrintController PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(PrintVisualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(PayloadPositionPublisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(PrintController PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(PrintVisualization PUBLIC c_std_99 cxx_std_17)
target_compile_features(PayloadPositionPublisher PUBLIC c_std_99 cxx_std_17)
install(TARGETS PrintController
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS PrintVisualization
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS PayloadPositionPublisher
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
