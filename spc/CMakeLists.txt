

add_subdirectory(time_series)
add_subdirectory(common)
add_subdirectory(elements)
add_subdirectory(geology)
add_subdirectory(methods)
add_subdirectory(io)
add_subdirectory(devices)

if (BUILD_PYTHON_WRAPPINGS)
    add_subdirectory(python_interface)
endif()

if(BUILD_MICMAC_RELATED)
   add_subdirectory(micmac)
endif()

if(BUILD_SPC_DOC)
    add_subdirectory(doc)
endif()

if(SWIG_WRAPPINGS)
    add_subdirectory(wrappings)
endif()


