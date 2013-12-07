cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(SPC)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER) #in lowercase

#supress warnings from PCL about SHOT etc... - not working!
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

set(SPC_LIBRARIES CACHE INTERNAL "SPC libs" FORCE)

option(BUILD_SPC_SHARED "Build spc library as a shared lib." ON)
option(BUILD_TOOLS "Build the tools." OFF)
option(BUILD_SANDBOX_TOOLS "Build sandbox tools." OFF)
option(BUILD_MICMAC_RELATED "Build MICMAC-related code." OFF)
option(BUILD_PYTHON_WRAPPINGS "Build python interface for the library. Only some classes have been wrapped!" OFF)
option(BUILD_SPC_DOC "Build SPC doc" OFF)
option(BUILD_TESTS "Do tests" OFF)
option(SWIG_WRAPPINGS "Use swigs to generate wrappings for some methods" OFF)
option(USE_PARALLEL_OPENMP "Enable paralellization of some algorithms via OpenMP" OFF)
option(BUILD_TESTING_TOOLS "Build the tools contained into the tests folder" OFF)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#PCL!
find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
message("${PCL_INCLUDE_DIRS}")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Standard set of flags for c++
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -std=c++11 -fopenmp -fPIC -lpthread -lgcc_s")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -pipe -std=c++0x -fPIC " )

#add custom functions and macros
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake)

# include the current source dir
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(USE_PARALLEL_OPENMP) # a definition if use parallelization where possible
    add_definitions(-DUSE_OPENMP)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -lpthread -lgcc_s" )
endif()


#micmac lib part
if(BUILD_MICMAC_RELATED)
    add_subdirectory(external)
    set(CULTURE_3D_BUILD_DIR "/home/luca/Code/culture3d/build" CACHE STRING "path where a culture3d build has been compiled is located. This is my default. you should change to your needs")
    set(CULTURE_3D_SOURCE_DIR "/home/luca/Code/culture3d" CACHE STRING "path where culture 3d source resides. This is my default. Change o your needs")
    link_directories(${CULTURE_3D_BUILD_DIR}/src)
endif()


# the main libraries
add_subdirectory(spc)

#also tools if needed
if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

#also tools if needed
if(BUILD_SANDBOX_TOOLS)
    add_subdirectory(sandbox)
endif()

#also tools if needed
if(BUILD_TESTING_TOOLS)
    add_subdirectory(tests)
endif()
