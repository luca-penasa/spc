cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(spc)

include(CompilerOptions.cmake)
include(Versioning.cmake)


INCLUDE(cmake/UpdateCacheVariable.cmake) #courtesy of ceres-solver

#my definitions
set(SPC_DEFINITIONS "")

# my libs
set(SPC_LIBRARIES CACHE INTERNAL "SPC libs" FORCE) #this will collect all the modular libs spc will create

# all libraries into a single dir for the build tree
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_subdirectory(submodules)



############################# OPTIONS ######################################################################
if (UNIX)
    option(SPC_BUILD_SHARED "Build spc library as a shared lib." ON)
endif()




#option(SPC_BUILD_SANDBOX_TOOLS "Build sandbox tools." OFF)
option(SPC_BUILD_EXAMPLES_TOOLS "Build examples tools." OFF)

option(SPC_USE_IWYU "Use include-what-you-use to notify about unused includes" OFF)

OPTION(SPC_GFLAGS "Enable Google Flags. Needed by the tools" ON)
option(SPC_BUILD_TESTS "Do tests" OFF)
option(SPC_ENABLE_INSTALL "Enable/Disable the install target" OFF)
OPTION(SPC_MINIGLOG "Use a stripped down version of glog." ON)

option(SPC_WITH_PCL "Compile SPC lib with PCL support - Many more dependencies" OFF)

if (SPC_WITH_PCL)
    option(SPC_BUILD_TOOLS "Build the tools." ON)
endif()

#option(SPC_GFLAGS_IN_NAMESPACE_GOOGLE "says if gflags is expected to be into the google namespace" OFF)


option(SPC_USE_PARALLEL_OPENMP "Enable paralellization of some algorithms via OpenMP" OFF)
option(SPC_BUILD_TESTING_TOOLS "Build the tools contained into the tests folder" OFF)
#############################################################################################################

#


#add custom functions and macros
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake)

# include the current source dir
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


include(FindExternal.cmake)


#micmac lib part
if(SPC_BUILD_MICMAC_RELATED)
    #    add_subdirectory(external)
    set(CULTURE_3D_BUILD_DIR "/home/luca/Code/culture3d/build" CACHE STRING "path where a culture3d build has been compiled is located. This is my default. you should change to your needs")
    set(CULTURE_3D_SOURCE_DIR "/home/luca/Code/culture3d" CACHE STRING "path where culture 3d source resides. This is my default. Change o your needs")
    link_directories(${CULTURE_3D_BUILD_DIR}/src)
endif()

##################### ADD MY DEFINITIONS #################
add_definitions(${SPC_DEFINITIONS})
#########################################################


# the main libraries
add_subdirectory(spc)

#also tools if needed
if(SPC_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

#also tools if needed
if(SPC_BUILD_SANDBOX_TOOLS)
    add_subdirectory(sandbox)
endif()

#also tools if needed
if(SPC_BUILD_EXAMPLES_TOOLS)
    add_subdirectory(examples)
endif()

#also tools if needed
if(SPC_BUILD_TESTING_TOOLS)
    add_subdirectory(tests)
endif()

if(SPC_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(SPC_USE_IWYU)
    find_program(iwyu_path NAMES include-what-you-use iwyu)
    if(NOT iwyu_path)
        message(FATAL_ERROR "Could not find the program include-what-you-use")
    endif()
endif()

INCLUDE(PackageExport.cmake)
