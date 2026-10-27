project(CFE_RACS2_BRIDGE_CLIENT C)

add_cfe_app(racs2_bridge_client fsw/src/racs2_bridge_client.c)
target_include_directories(racs2_bridge_client PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../racs2_common)
target_compile_definitions(racs2_bridge_client PRIVATE _DEFAULT_SOURCE)
target_link_libraries(racs2_bridge_client websockets)
