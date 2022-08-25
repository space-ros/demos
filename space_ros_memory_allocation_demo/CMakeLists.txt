cmake_minimum_required(VERSION 3.8)
project(space_ros_memory_allocation_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(osrf_testing_tools_cpp REQUIRED)

add_executable(talker_main src/talker_main.cpp)
target_include_directories(talker_main PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(talker_main osrf_testing_tools_cpp::memory_tools)
ament_target_dependencies(talker_main
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "std_msgs")

add_executable(listener_main src/listener_main.cpp)
target_include_directories(listener_main PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(listener_main osrf_testing_tools_cpp::memory_tools)
ament_target_dependencies(listener_main
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "std_msgs")

add_library(${PROJECT_NAME}_library SHARED src/talker_library.cpp src/listener_library.cpp)
target_include_directories(${PROJECT_NAME}_library PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_definitions(${PROJECT_NAME}_library
  PRIVATE "SPACE_ROS_MEMORY_ALLOCATION_DEMO_BUILDING_DLL")
target_link_libraries(${PROJECT_NAME}_library osrf_testing_tools_cpp::memory_tools)
ament_target_dependencies(${PROJECT_NAME}_library
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "std_msgs")

rclcpp_components_register_node(${PROJECT_NAME}_library
  PLUGIN "space_ros_memory_allocation_demo::Talker"
  EXECUTABLE talker)
rclcpp_components_register_node(${PROJECT_NAME}_library
  PLUGIN "space_ros_memory_allocation_demo::Listener"
  EXECUTABLE listener)

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

install(TARGETS
  ${PROJECT_NAME}_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS talker_main listener_main
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
