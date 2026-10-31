# The SCH_LAB and TO_LAB tables from BPNode's dtn_defs reference message IDs of
# other apps, so give those table builds the other apps' include directories.
foreach(TBL sch_lab.table to_lab.table)
  target_include_directories(${TBL} INTERFACE
    $<TARGET_PROPERTY:ci_lab,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:to_lab,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:sample_app,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:cf,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:bpnode,INCLUDE_DIRECTORIES>
  )
endforeach()
