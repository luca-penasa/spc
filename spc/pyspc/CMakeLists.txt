
#link_directories(${CULTURE_3D_BUILD_DIR}/src)
#include_directories("${CULTURE_3D_SOURCE_DIR}/include")
#include_directories("${CULTURE_3D_SOURCE_DIR}/src/uti_image/Ann")

#include_directories(${CULTURE_3D_BUILD_DIR}/include)

#find_package(X11)

#SET(CMAKE_BUILD_TYPE "DEBUG")

#find_package(PythonLibs 2.7 REQUIRED)
#include_directories(${PYTHON_INCLUDE_DIRS})
#include_directories(${CMAKE_SOURCE_DIR}/external/Boost.NumPy)


#FIND_PACKAGE(Boost 1.45.0)
#IF(Boost_FOUND)
#  INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}")
#  SET(Boost_USE_STATIC_LIBS OFF)
#  SET(Boost_USE_MULTITHREADED ON)
#  SET(Boost_USE_STATIC_RUNTIME OFF)
#  FIND_PACKAGE(Boost 1.45.0 COMPONENTS python)

#ELSEIF(NOT Boost_FOUND)
#  MESSAGE(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?")
#ENDIF()

FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)
FIND_PACKAGE(Boost COMPONENTS python)



ENABLE_TESTING()
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
LINK_LIBRARIES(${Boost_LIBRARIES} ${PYTHON_LIBRARIES}) # Deprecated but so convenient!

PYTHON_ADD_MODULE(pyspc pymm.cpp pymm.h  eigen_numpy.h  eigen_numpy.cc)

include_directories()

#spc_add_library()
#set(additional_libs ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} boost_numpy spc_elements)

#add_library(pyspc SHARED pymm.cpp pymm.h)
target_link_libraries(pyspc ${Boost_LIBRARIES}  boost_numpy ${SPC_LIBRARIES})
#spc_install_target_library_libs(pyspc)







