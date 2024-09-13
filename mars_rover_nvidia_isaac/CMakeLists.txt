cmake_minimum_required(VERSION 3.8)
project(mars_rover_nvidia_isaac)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(control_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
# find_package(simulation REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  mars_rover_nvidia_isaac/isaac_controller_loader.py
  mars_rover_nvidia_isaac/nodes/wheel_velocity_controller.py
  mars_rover_nvidia_isaac/nodes/steer_position_controller.py
  mars_rover_nvidia_isaac/nodes/arm_joint_trajectory_controller.py
  mars_rover_nvidia_isaac/nodes/mast_joint_trajectory_controller.py
  mars_rover_nvidia_isaac/nodes/wheel_tree_position_controller.py
  mars_rover_nvidia_isaac/nodes/run_isaac_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
