
# ORDER MATTERS: the next library will be linked againte the previous ones only!
# Note: not a good way of doing this, we should add a kind of dependency between modules,
# so that one may possibly disable/enable certain modules

add_subdirectory(external)
add_subdirectory(elements)
add_subdirectory(io)
add_subdirectory(methods)



if (SPC_BUILD_CALIBRATION)
    add_subdirectory(calibration)
endif()





if(BUILD_EXPERIMENTAL)
    add_subdirectory(experimental)
endif()


if (SPC_BUILD_PYTHON_WRAPPINGS)
    add_subdirectory(python_interface)
endif()

if(SPC_BUILD_MICMAC_RELATED)
   add_subdirectory(micmac)
endif()

if(SPC_BUILD_DOC)
    add_subdirectory(doc)
endif()

if(SPC_SWIG_WRAPPINGS)
    add_subdirectory(wrappings)
endif()

if(SPC_BUILD_TESTS)
    add_subdirectory(tests)
endif()

