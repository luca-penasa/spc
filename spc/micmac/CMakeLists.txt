

include_directories("${CULTURE_3D_SOURCE_DIR}/include")
include_directories(${CULTURE_3D_BUILD_DIR}/include)

find_package(X11)

#SET(CMAKE_BUILD_TYPE "DEBUG")

find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/external/Boost.NumPy)


FIND_PACKAGE(Boost 1.45.0)

IF(Boost_FOUND)
  INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}")
  SET(Boost_USE_STATIC_LIBS OFF)
  SET(Boost_USE_MULTITHREADED ON)
  SET(Boost_USE_STATIC_RUNTIME OFF)
  FIND_PACKAGE(Boost 1.45.0 COMPONENTS python)

ELSEIF(NOT Boost_FOUND)
  MESSAGE(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?")
ENDIF()
    link_directories(${CULTURE_3D_BUILD_DIR}/src)
set(additional_libraries ${X11_LIBRARIES} elise)
spc_add_library(micmac "${additional_libraries}")




