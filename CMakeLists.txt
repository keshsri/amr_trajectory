cmake_minimum_required(VERSION 3.8)
project(trajectory_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
)

add_executable(trajectory_publisher src/trajectory_publisher.cpp)
#target_link_libraries(trajectory_publisher rclcpp tf2_ros)
ament_target_dependencies(trajectory_publisher rclcpp std_msgs visualization_msgs tf2_ros nav_msgs)

add_executable(trajectory_reader src/trajectory_reader.cpp)
#target_link_libraries(trajectory_reader rclcpp tf2_ros)
ament_target_dependencies(trajectory_reader rclcpp std_msgs visualization_msgs tf2_ros nav_msgs nlohmann_json)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(trajectory_publisher "${cpp_typesupport_target}")
target_link_libraries(trajectory_reader "${cpp_typesupport_target}")

install(TARGETS
  trajectory_publisher
  trajectory_reader
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  src/odom_publisher.py
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
