project(CFE_RUN_APP C)

add_cfe_app(run_app
  fsw/src/run_app.c
  fsw/src/RACS2Bridge_geometry_msgs.pb-c.c
)
target_include_directories(run_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../racs2_common)
target_compile_definitions(run_app PRIVATE _DEFAULT_SOURCE)
target_link_libraries(run_app protobuf-c)
