
# ORDER MATTERS: the next library will be linked againte the previous ones only!
# Note: not a good way of doing this, we should add a kind of dependency between modules,
# so that one may possibly disable/enable certain modules

## these core modules are always build





option(SPC_MODULE_CERES_CALIBRATION "Build experimental code nonlinear intensity-calibration support" OFF)
#option(SPC_MODULE_EXPERIMENTAL "Build experimental module." OFF)
option(SPC_MODULE_MICMAC "Build MICMAC-related code." OFF)
#option(SPC_MODULE_PYTHON_WRAPPINGS "Build python interface for the library. Only some classes have been wrapped!" OFF)


option(SPC_MODULE_PYSPC "Build python interface for the library. Only some classes have been wrapped!" OFF)


#option(SPC_BUILD_TESTS "Do tests" OFF)
option(SPC_BUILD_DOC "Build SPC doc" OFF)

#option(SPC_BUILD_WRAPPINGS "Use swigs to generate wrappings for some methods" OFF)



add_subdirectory(external)
add_subdirectory(core)

add_subdirectory(elements)

add_subdirectory(io)



add_subdirectory(methods)

#add_subdirectory(templated)


if(SPC_MODULE_CERES_CALIBRATION)
    add_subdirectory(ceres_calibration)
endif()


if(SPC_MODULE_PYSPC)
    add_subdirectory(pyspc)
endif()

if(SPC_MODULE_MICMAC)
   add_subdirectory(micmac)
endif()




#if(SPC_BUILD_TESTS)
#    add_subdirectory(tests)
#endif()

if(SPC_BUILD_DOC)
    add_subdirectory(doc)
endif()

