cmake_minimum_required(VERSION 3.8)
project(mars_rover_teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

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

# Add RViz plugin
add_library(${PROJECT_NAME} SHARED
  src/mars_rover_teleop.cpp
)

target_link_libraries(${PROJECT_NAME} 
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rclcpp::rclcpp
  pluginlib::pluginlib
)

ament_target_dependencies(${PROJECT_NAME}
  "rviz_common"
  "rviz_rendering"
  "rclcpp"
  "std_srvs"
  "pluginlib"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_libraries(${PROJECT_NAME})

ament_package()
