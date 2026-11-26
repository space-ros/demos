# cFS targets for the DTN Curiosity rover demo, based on the cFE sample_defs
# targets.cmake with BPNode added (see nasa/bp dtn_defs/targets.cmake).
#
#   cpu1: ground node (ipn:100)
#   cpu2: rover node  (ipn:200)

SET(MISSION_NAME "DTN")
SET(SPACECRAFT_ID 0x42)

# sample_app/sample_lib stay in the build because BPNode's TO_LAB table
# references sample_app's message IDs.
list(APPEND MISSION_GLOBAL_APPLIST sample_app sample_lib)
list(APPEND MISSION_GLOBAL_APPLIST ci_lab to_lab sch_lab)
list(APPEND MISSION_GLOBAL_APPLIST bplib bpnode cf)

SET(FT_INSTALL_SUBDIR "host/functional-test")

SET(MISSION_CPUNAMES cpu1 cpu2)

foreach(CPU cpu1 cpu2)
    SET(${CPU}_FILELIST cfe_es_startup.scr)
    SET(${CPU}_PSP_MODULELIST udpsock_intf)
    SET(${CPU}_SYSTEM i686-linux-gnu)
endforeach()
SET(cpu1_PROCESSORID 1)
SET(cpu1_APPLIST run_app)

SET(cpu2_PROCESSORID 2)
SET(cpu2_APPLIST racs2_bridge_client)
LIST(APPEND cpu2_FILELIST racs2_bridge_config.txt)
