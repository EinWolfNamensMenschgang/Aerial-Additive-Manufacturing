cmake_minimum_required(VERSION 3.8)
project(gcode_to_path)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/PrintPose.msg"
  "msg/PrintPath.msg"
)
add_executable(PathFromGcode src/PathFromGcode.cpp)
ament_target_dependencies(PathFromGcode rclcpp nav_msgs)
target_include_directories(PathFromGcode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(PathFromGcode PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS PathFromGcode
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

rosidl_generate_interfaces(gcode_to_path
  "msg/PrintPose.msg"
  "msg/PrintPath.msg"
  DEPENDENCIES nav_msgs
  )
ament_export_dependencies(rosidl_default_runtime)
rosidl_target_interfaces(PathFromGcode
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_package()
