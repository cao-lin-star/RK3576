cmake_minimum_required(VERSION 3.8)
project(rk3576_footbath_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(low_lidar_angular_filter_lib src/low_lidar_angular_filter.cpp)
target_include_directories(low_lidar_angular_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(low_lidar_angular_filter_lib sensor_msgs)

add_executable(low_lidar_angular_filter src/low_lidar_angular_filter_node.cpp)
target_link_libraries(low_lidar_angular_filter low_lidar_angular_filter_lib)
ament_target_dependencies(low_lidar_angular_filter rclcpp sensor_msgs)

add_library(dual_lidar_fusion_lib src/dual_lidar_fusion.cpp)
target_include_directories(dual_lidar_fusion_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dual_lidar_fusion_lib sensor_msgs)

add_executable(dual_lidar_mapping_fusion src/dual_lidar_fusion_node.cpp)
target_link_libraries(dual_lidar_mapping_fusion dual_lidar_fusion_lib)
ament_target_dependencies(
  dual_lidar_mapping_fusion
  diagnostic_msgs rclcpp sensor_msgs tf2 tf2_ros)

install(TARGETS low_lidar_angular_filter_lib low_lidar_angular_filter
  EXPORT export_low_lidar_angular_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS dual_lidar_fusion_lib dual_lidar_mapping_fusion
  EXPORT export_low_lidar_angular_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

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

  ament_add_gtest(test_dual_lidar_fusion test/test_dual_lidar_fusion.cpp)
  if(TARGET test_dual_lidar_fusion)
    target_link_libraries(test_dual_lidar_fusion dual_lidar_fusion_lib)
    ament_target_dependencies(test_dual_lidar_fusion sensor_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_low_lidar_angular_filter HAS_LIBRARY_TARGET)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(diagnostic_msgs tf2 tf2_ros)
ament_package()
