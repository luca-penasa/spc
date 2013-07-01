########### USING SWIG TRY
FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})

FIND_PACKAGE(PythonLibs)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})


INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})



SET(CMAKE_SWIG_FLAGS "")

if(BUILD_MICMAC_RELATED)
    include_directories("${CULTURE_3D_SOURCE_DIR}/include")
    include_directories(${CULTURE_3D_BUILD_DIR}/include)
    include_directories(${PYTHON_INCLUDE_DIRS})
    include_directories(${CMAKE_SOURCE_DIR}/external/Boost.NumPy)

    execute_process ( COMMAND python -c "from distutils.sysconfig import get_python_lib; print get_python_lib()" OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)
    message("${PYTHON_SITE_PACKAGES}")

    include_directories(../micmac/)
include_directories(${PYTHON_SITE_PACKAGES}/numpy/core/include/)
message(${PYTHON_INCLUDE_DIRS})

#   SET_SOURCE_FILES_PROPERTIES(../micmac/micmac.i PROPERTIES CPLUSPLUS ON)
#    #SET_SOURCE_FILES_PROPERTIES(../micmac/micmac.i PROPERTIES SWIG_FLAGS)
#    #SET_SOURCE_FILES_PROPERTIES(../micmac/micmac.i PROPERTIES SWIG_FLAGS "-includeall")
#   SWIG_ADD_MODULE(micmac python ../micmac/micmac.i ../micmac/mm_utils.cpp)
#    SWIG_LINK_LIBRARIES(micmac ${PYTHON_LIBRARIES} spc_micmac)

endif()

 #   SET_SOURCE_FILES_PROPERTIES(../time_series/interface.i PROPERTIES CPLUSPLUS ON)
    #SET_SOURCE_FILES_PROPERTIES(../micmac/micmac.i PROPERTIES SWIG_FLAGS)
    #SET_SOURCE_FILES_PROPERTIES(../micmac/micmac.i PROPERTIES SWIG_FLAGS "-includeall")
#    SWIG_ADD_MODULE(time_series python ../time_series/interface.i )
#    SWIG_LINK_LIBRARIES(time_series ${PYTHON_LIBRARIES} spc_time_series)
