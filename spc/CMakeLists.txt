add_subdirectory(time_series)
add_subdirectory(common)
add_subdirectory(geology)
add_subdirectory(methods)

if (BUILD_PYTHON_WRAPPINGS)
    add_subdirectory(python_interface)
endif()

if(BUILD_MICMAC_RELATED)
   add_subdirectory(micmac)
endif()

if(BUILD_SPC_DOC)
    add_subdirectory(doc)
endif()
