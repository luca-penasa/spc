
# ORDER MATTERS: the next library will be linked againte the previous ones only!
# Note: not a good way of doing this, we should add a kind of dependency between modules,
# so that one may possibly disable/enable certain modules


add_subdirectory(common)
add_subdirectory(elements)
add_subdirectory(time_series)
add_subdirectory(tables)
add_subdirectory(io)
add_subdirectory(flann)
add_subdirectory(scalar_fields_generators)
add_subdirectory(estimators)
add_subdirectory(methods)
add_subdirectory(devices)
add_subdirectory(calibration)
add_subdirectory(external)





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

