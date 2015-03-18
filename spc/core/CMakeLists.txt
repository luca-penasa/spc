if(SPC_MINIGLOG)
    message("compiling miniglog within spc_core")
    set(ADDITIONAL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../../miniglog/glog/logging.cc")
    message("add source: " ${ADDITIONAL_SOURCES})
endif(SPC_MINIGLOG)

spc_add_library()






