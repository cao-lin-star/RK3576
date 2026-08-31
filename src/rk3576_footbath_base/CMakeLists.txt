cmake_minimum_required(VERSION 3.8)
project(rk3576_footbath_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(footbath_protocol src/protocol.cpp)
target_include_directories(footbath_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(stm32_serial_bridge src/stm32_serial_bridge.cpp)
target_link_libraries(stm32_serial_bridge footbath_protocol)
ament_target_dependencies(stm32_serial_bridge
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros diagnostic_msgs)

install(TARGETS footbath_protocol stm32_serial_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol footbath_protocol)
  target_include_directories(test_protocol PRIVATE include)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(footbath_protocol)
ament_package()
