cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(spc)


option(BUILD_WITH_CLANG "Build spc library using clang isntead of gcc." OFF)
if(BUILD_WITH_CLANG)
    message("Using Clang as compiler! This may not work properly")
    set(CMAKE_C_COMPILER "clang")
    set(CMAKE_CXX_COMPILER "clang++")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################### VERSIONING ###############################################
set(SPC_MAJOR_VERSION 0)
set(SPC_MINOR_VERSION 1)
set(SPC_PATCH_VERSION 0)
set(SPC_VERSION ${SPC_MAJOR_VERSION}.${SPC_MINOR_VERSION}.${SPC_PATCH_VERSION})
###############################################################################


########################### COMPILER FLAGS ##################################
#supress warnings from PCL about SHOT etc... - not working!
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -std=c++0x -fopenmp -fPIC -lpthread -lgcc_s")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -pipe -std=c++0x -fPIC -Wall -Wno-sign-compare" )
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -std=c++0x -Wall -Wno-long-long -pedantic" )

if (WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" )
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11 -fopenmp" )
endif()



##############################################################################


#FIND_PACKAGE( OpenMP REQUIRED)
#if(OPENMP_FOUND)
#    message("OPENMP FOUND")
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()

#find_package(OpenMP)
#include_directories("{OpenMP_INCLUDE_DIRS}")
#message(${OpenMP_INCLUDE_DIR})

include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})



############################################################################################################
set(SPC_LIBRARIES CACHE INTERNAL "SPC libs" FORCE) #this will collect all the modular libs spc will create
set(PCL_DEFINITIONS "") #my definitions
############################################################################################################

############################# OPTIONS ######################################################################
option(BUILD_SPC_SHARED "Build spc library as a shared lib." ON)
option(BUILD_TOOLS "Build the tools." OFF)
option(BUILD_TOOLS "Build really experimental staff." OFF)
option(BUILD_SANDBOX_TOOLS "Build sandbox tools." OFF)
option(BUILD_MICMAC_RELATED "Build MICMAC-related code." OFF)
option(BUILD_PYTHON_WRAPPINGS "Build python interface for the library. Only some classes have been wrapped!" OFF)
option(BUILD_SPC_DOC "Build SPC doc" OFF)
option(BUILD_TESTS "Do tests" OFF)
option(SWIG_WRAPPINGS "Use swigs to generate wrappings for some methods" OFF)
option(USE_PARALLEL_OPENMP "Enable paralellization of some algorithms via OpenMP" OFF)
option(BUILD_TESTING_TOOLS "Build the tools contained into the tests folder" OFF)
#############################################################################################################

################################### INSTALL OPTIONS #####################################################
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")


if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/spc)
endif()

set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH  "Installation directory for CMake files")


#make relative path absolute
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()
############################################################################################################

#########################  PCL! - everybody here will need it ###############################
find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
message("${PCL_INCLUDE_DIRS}")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
##################################################################################################

message("TESTING PCL !")
#define the PCL_VER_1_6_OR_OLDER preprocessor to compile qPCL with older versions of PCL
if ( PCL_VERSION VERSION_LESS  1.7 ) # VERSION_GREATER Works just like "greater or equal"        		
		message("You are using PCL 1.6 - the 1.7 is out !")
		set(SPC_DEFINITIONS ${SPC_DEFINITIONS} "-DPCL_VER_LESS_1_7")
endif()

#add custom functions and macros
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake)

# include the current source dir
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(USE_PARALLEL_OPENMP) # a definition if use parallelization where possible
    set(SPC_DEFINITIONS ${SPC_DEFINITIONS} "-DUSE_OPENMP")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -lpthread -lgcc_s" )
endif()




#micmac lib part
if(BUILD_MICMAC_RELATED)
    add_subdirectory(external)
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


message("${PCL_DEFINITIONS}")
message("${PCL_LIBRARIES}")

# The interesting stuff goes here
# ===============================

# Add all targets to the build-tree export set
export(TARGETS ${SPC_LIBRARIES} FILE "${PROJECT_BINARY_DIR}/SPCTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE SPC)

file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")

# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(SPCConfig.cmake.in "${PROJECT_BINARY_DIR}/SPCConfig.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "\${SPC_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(SPCConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/SPCConfig.cmake" @ONLY)
# ... for both
configure_file(SPCConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/SPCConfigVersion.cmake" @ONLY)

# Install the FooBarConfig.cmake and FooBarConfigVersion.cmake
install(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/SPCConfig.cmake"
              "${PROJECT_BINARY_DIR}/SPCConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

## Install the export set for use with the install-tree
#install(EXPORT SpcTargets DESTINATION
#               "${INSTALL_CMAKE_DIR}" COMPONENT dev)
