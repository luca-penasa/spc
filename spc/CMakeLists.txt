
# ORDER MATTERS: the next library will be linked againte the previous ones only!
# Note: not a good way of doing this, we should add a kind of dependency between modules,
# so that one may possibly disable/enable certain modules

add_subdirectory(external)
add_subdirectory(elements)
add_subdirectory(io)
add_subdirectory(methods)









if(BUILD_EXPERIMENTAL)
    add_subdirectory(experimental)
endif()


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

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

