cmake_minimum_required(VERSION 3.8)
project(canadarm_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/MoveJoint.action"
# )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  action
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
