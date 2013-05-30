
spc_library_needs_pcl()
set(additional_libraries ${additional_libraries} ${SPC_LIBRARIES})

spc_add_library(common "${additional_libraries}")



