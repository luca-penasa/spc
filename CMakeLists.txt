cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(SPC)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER) #in lowercase

set(SPC_LIBRARIES CACHE INTERNAL "SPC libs" FORCE)

option(BUILD_SPC_SHARED "Build spc library as a shared lib." ON)
option(BUILD_TOOLS "Build the tools." ON)
option(BUILD_MICMAC_RELATED "Build MICMAC-related code." ON)
option(BUILD_PYTHON_WRAPPINGS "Build python interface for the library. Only some classes have been wrapped!" ON)
option(BUILD_SPC_DOC "Build SPC doc" OFF)
option(BUILD_QGEO_CC_PLUGIN "Build SPC-based plugin for CloudCompare" ON)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_USE_NEW_STDVECTOR
                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)



# Standard set of flags for c++
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -std=c++11 -fopenmp -fPIC")
set(CMAKE_BUILD_TYPE DEBUG) #force it to debug for now

#add custom functions and macros
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake)

# includes starts from here
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# the main libraries
add_subdirectory(${PROJECT_NAME_LOWER})

if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(BUILD_MICMAC_RELATED)
    add_subdirectory(external)
    set(CULTURE_3D_BUILD_DIR "/home/luca/Code/culture3d/build" CACHE STRING "path where a culture3d build has been compiled is located. This is my default. you should change to your needs")
    set(CULTURE_3D_SOURCE_DIR "/home/luca/Code/culture3d" CACHE STRING "path where culture 3d source resides. This is my default. Change o your needs")
endif()


if(BUILD_QGEO_CC_PLUGIN)
    set(CC_BUILD_DIR "/home/luca/Code/CC_git_clean/build" CACHE STRING "Dir with CloudCompare build")
    set(CC_SOURCE_DIR "/home/luca/Code/CC_git_clean" CACHE STRING "Dir with CloudCompare souces")
    set(QGEO_INSTALL_DIR "/home/luca/apps/bin/plugins" CACHE STRING "Where to intall the qGEO plugin")
add_subdirectory(qGEO)

endif()


