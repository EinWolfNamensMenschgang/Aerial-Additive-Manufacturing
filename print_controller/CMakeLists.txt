cmake_minimum_required(VERSION 3.8)
project(print_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gcode_to_path REQUIRED)
find_package(message_filters REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executables
add_executable(PrintController src/PrintController.cpp)
add_executable(PrintControllerSingle src/PrintControllerSingle.cpp)
add_executable(PrintVisualization src/PrintVisualization.cpp)
add_executable(PrintVisualizationSingle src/PrintVisualizationSingle.cpp)
add_executable(PayloadPositionPublisher src/PayloadPositionPublisher.cpp)
add_executable(MapGenerator src/map_generator.cpp)

# Link dependencies
ament_target_dependencies(PrintController rclcpp geometry_msgs visualization_msgs gcode_to_path)
ament_target_dependencies(PrintControllerSingle rclcpp geometry_msgs visualization_msgs gcode_to_path tf2 tf2_geometry_msgs)
ament_target_dependencies(PrintVisualization rclcpp geometry_msgs visualization_msgs gcode_to_path)
ament_target_dependencies(PrintVisualizationSingle rclcpp geometry_msgs visualization_msgs gcode_to_path)
ament_target_dependencies(PayloadPositionPublisher rclcpp nav_msgs message_filters)
target_link_libraries(MapGenerator yaml-cpp)

# Include directories
target_include_directories(PrintController PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(PrintVisualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(PayloadPositionPublisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set compile features
target_compile_features(PrintController PUBLIC c_std_99 cxx_std_17)
target_compile_features(PrintVisualization PUBLIC c_std_99 cxx_std_17)
target_compile_features(PayloadPositionPublisher PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS PrintController
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS PrintControllerSingle
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS PrintVisualization
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS PrintVisualizationSingle
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS PayloadPositionPublisher
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS MapGenerator
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_copyright_FOUND TRUE)  # Skip linter for copyrights
  set(ament_cmake_cpplint_FOUND TRUE)  # Skip cpplint (only works in a git repo)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

