cmake_minimum_required(VERSION 3.8)
project(rk3576_footbath_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(low_lidar_angular_filter_lib src/low_lidar_angular_filter.cpp)
target_include_directories(low_lidar_angular_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(low_lidar_angular_filter_lib sensor_msgs)

add_executable(low_lidar_angular_filter src/low_lidar_angular_filter_node.cpp)
target_link_libraries(low_lidar_angular_filter low_lidar_angular_filter_lib)
ament_target_dependencies(low_lidar_angular_filter rclcpp sensor_msgs)

install(TARGETS low_lidar_angular_filter_lib low_lidar_angular_filter
  EXPORT export_low_lidar_angular_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_low_lidar_angular_filter test/test_low_lidar_angular_filter.cpp)
  if(TARGET test_low_lidar_angular_filter)
    target_link_libraries(test_low_lidar_angular_filter low_lidar_angular_filter_lib)
    ament_target_dependencies(test_low_lidar_angular_filter sensor_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_low_lidar_angular_filter HAS_LIBRARY_TARGET)
ament_export_dependencies(sensor_msgs)
ament_package()
