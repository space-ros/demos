cmake_minimum_required(VERSION 3.5)
project(spaceros_procgen_envs)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

## Install Python package
ament_python_install_package(spaceros_procgen_envs)

## Install Python scripts
set(SCRIPTS_DIR scripts)
install(PROGRAMS
    ${SCRIPTS_DIR}/blender/procgen_assets.py
    ${SCRIPTS_DIR}/agent_random.py
    ${SCRIPTS_DIR}/agent_zero.py
    ${SCRIPTS_DIR}/clean_procgen_cache.py
    ${SCRIPTS_DIR}/list_envs.py
    ${SCRIPTS_DIR}/run_spaceros.py
    ${SCRIPTS_DIR}/teleop.py
    DESTINATION lib/${PROJECT_NAME}
)

## Install directories
install(DIRECTORY apps config launch DESTINATION share/${PROJECT_NAME})

## Setup the project
ament_package()
