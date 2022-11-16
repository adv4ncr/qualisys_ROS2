cmake_minimum_required(VERSION 3.8)
project(qualisys_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(qualisys_cpp_sdk REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

add_executable(qualisys_adapter src/qualisys_adapter.cpp)

## COMPILE
target_include_directories(qualisys_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# https://github.com/ament/ament_cmake/issues/292
target_link_libraries(qualisys_adapter
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  qualisys_cpp_sdk
)


ament_target_dependencies(qualisys_adapter
  geometry_msgs
  tf2
  tf2_ros
)

target_compile_features(qualisys_adapter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# INSTALL
install(TARGETS qualisys_adapter
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
