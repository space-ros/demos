cmake_minimum_required(VERSION 3.5)
project(helicopter_flight_control)

# Default to C11 for C and C++17 for C++
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED) # Assuming using Ignition Fortress
find_package(pluginlib REQUIRED)

if("$ENV{IGNITION_VERSION}" STREQUAL "citadel")
  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Citadel")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "edifice")
  find_package(ignition-gazebo5 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Edifice")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Fortress")

else()
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Fortress")
endif()

find_package(ignition-plugin1 REQUIRED)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

# Include directories
include_directories(include)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/helicopter_flight_control.cc
)


target_link_libraries(${PROJECT_NAME}
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-plugin${IGN_PLUGIN_VER}::register
)

# Ament target dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(helicopter_flight_control ign_hardware_plugins.xml)


# Install launch files and other directories if necessary
# e.g., install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Setup the project
ament_package()
