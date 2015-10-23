cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(spc)


INCLUDE(cmake/UpdateCacheVariable.cmake) #courtesy of ceres-solver

set(SPC_DEFINITIONS "") #my definitions

#find_package(Qt5Widgets)

# all libraries into a single dir for the build tree
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)


LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(UNIX)
    add_definitions(-fPIC)
endif()

option(SPC_BUILD_WITH_CLANG "Build spc library using clang isntead of gcc." OFF)
if(SPC_BUILD_WITH_CLANG)
    # try to find clang executable!
    find_program(CLANG_CPP_EXECUTABLE
      NAMES clang++-3.7 clang++-3.6 clang++
      PATHS ENV)


    find_program(CLANG_EXECUTABLE
      NAMES clang-3.7 clang-3.6 clang
      PATHS ENV)

    message("CLANG_CPP ${CLANG_CPP_EXECUTABLE}")
    message("CLANG ${CLANG_EXECUTABLE}")

    if(CLANG_CPP_EXECUTABLE)
	  message("clang C++ and C compilers found : ${CLANG_CPP_EXECUTABLE}")
	  set(CMAKE_C_COMPILER ${CLANG_EXECUTABLE})
	  set(CMAKE_CXX_COMPILER ${CLANG_CPP_EXECUTABLE})
    else()
	  message("In else")
	  message(FATAL_ERROR "Can't found program: clang and clang++")
    endif()

    message(WARNING "Using Clang as compiler! This may not work properly")

endif()

#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################### VERSIONING ###############################################
set(SPC_MAJOR_VERSION 0)
set(SPC_MINOR_VERSION 1)
set(SPC_PATCH_VERSION 0)
set(SPC_VERSION ${SPC_MAJOR_VERSION}.${SPC_MINOR_VERSION}.${SPC_PATCH_VERSION})
###############################################################################


########################### COMPILER FLAGS ##################################
if (WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /bigobj" )
else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++0x -Wno-narrowing " )
endif()


if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()

add_subdirectory(submodules)


include_directories("${CMAKE_CURRENT_SOURCE_DIR}/submodules/nanoflann/include/")

############ CEREAL ####################################
set(CEREAL_INCLUDE_DIRS "submodules/cereal/include")



#right time to try to fine a system-wide install of cereal if any

include_directories(${CEREAL_INCLUDE_DIRS})

set( BOOL CEREAL_IS_INTERNAL TRUE)

set(CEREAL_IS_INTERNAL TRUE CACHE BOOL "if internal cereal is used")

######################################################



############################################################################################################
set(SPC_LIBRARIES CACHE INTERNAL "SPC libs" FORCE) #this will collect all the modular libs spc will create
############################################################################################################

############################# OPTIONS ######################################################################
if (UNIX)
    option(SPC_BUILD_SHARED "Build spc library as a shared lib." ON)
endif()

option(SPC_BUILD_WITH_GPROF "Build with -pg flags for profiling." OFF)



#option(SPC_BUILD_SANDBOX_TOOLS "Build sandbox tools." OFF)
option(SPC_BUILD_EXAMPLES_TOOLS "Build examples tools." OFF)

OPTION(SPC_GFLAGS "Enable Google Flags. Needed by the tools" ON)
option(SPC_BUILD_TESTS "Do tests" OFF)
option(SPC_ENABLE_INSTALL "Enable/Disable the install target" OFF)
OPTION(SPC_MINIGLOG "Use a stripped down version of glog." ON)

option(SPC_WITH_PCL "Compile SPC lib with PCL support - Many more dependencies" OFF)

if (SPC_WITH_PCL)
	option(SPC_BUILD_TOOLS "Build the tools." ON)
endif()

#option(SPC_GFLAGS_IN_NAMESPACE_GOOGLE "says if gflags is expected to be into the google namespace" OFF)


option(SPC_USE_PARALLEL_OPENMP "Enable paralellization of some algorithms via OpenMP" ON)
option(SPC_BUILD_TESTING_TOOLS "Build the tools contained into the tests folder" OFF)
#############################################################################################################

################################### INSTALL OPTIONS #####################################################
set(SPC_INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(SPC_INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(SPC_INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")

##### def dir for cmake files on install
if(WIN32 AND NOT CYGWIN)
  set(SPC_DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(SPC_DEF_INSTALL_CMAKE_DIR lib/CMake/SPC)
endif()

set(INSTALL_CMAKE_DIR ${SPC_DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var SPC_INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

message("INCLUDE ${SPC_INSTALL_INCLUDE_DIR}")
message("LIB ${SPC_INSTALL_LIB_DIR}")
message("BIN ${SPC_INSTALL_BIN_DIR}")
message("CMAKE ${SPC_INSTALL_CMAKE_DIR}")


if(SPC_USE_PARALLEL_OPENMP) # a definition if use parallelization where possible
    FIND_PACKAGE( OpenMP REQUIRED)
    if(OPENMP_FOUND)
        message("OPENMP FOUND")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
        set(SPC_DEFINITIONS ${SPC_DEFINITIONS} "-DUSE_OPENMP")
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -lpthread -lgcc_s" )
    endif()

endif()


############################################################################################################

#########################  PCL! - everybody here will need it ###############################
if (SPC_WITH_PCL)
    find_package(PCL 1.7 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    message("${PCL_INCLUDE_DIRS}")
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})


    message("TESTING PCL !")
    #define the PCL_VER_1_6_OR_OLDER preprocessor to compile qPCL with older versions of PCL
    if( PCL_VERSION VERSION_LESS  1.7 ) # VERSION_GREATER Works just like "greater or equal"
        message("You are using PCL 1.6 - the 1.7 is out !")
        set(SPC_DEFINITIONS ${SPC_DEFINITIONS} "-DPCL_VER_LESS_1_7 -DSPC_WITH_PCL")
    endif()

    list(APPEND SPC_DEFINITIONS -DSPC_WITH_PCL)
else()
    message(WARNING "Not compiling with PCL")
    # we need to find some requirements by hand
    find_package(Eigen3 REQUIRED)
    include_directories(${EIGEN3_INCLUDE_DIR})



    if(WIN32)
        # also boost! please use static libs as def on win so that we dont have to move boost libs along with the app
        set(Boost_USE_STATIC_LIBS   ON)
    endif()

    set(Boost_USE_MULTITHREADED ON)
    find_package(Boost COMPONENTS filesystem system REQUIRED)
    include_directories(${Boost_INCLUDE_DIRS})
    list(APPEND SPC_DEFINITIONS ${Boost_DEFINITIONS} ${EIGEN3_DEFINITIONS})
endif()
##################################################################################################

#add custom functions and macros
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake)

# include the current source dir
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# gflags is optional
if(SPC_GFLAGS)
    find_package(Gflags REQUIRED)

    if(GFLAGS_FOUND)
	message("GFLAGS FOUND")

	INCLUDE_DIRECTORIES(${GFLAGS_INCLUDE_DIRS})

	message("GFLAGS found and it is in namespace: " ${GFLAGS_NAMESPACE})

	if(GFLAGS_NAMESPACE STREQUAL "google")
	    set(SPC_DEFINITIONS ${SPC_DEFINITIONS} "-DGFLAGS_IN_NS_GOOGLE")
	endif()

    else(GFLAGS_FOUND)
	message("GLFAGS requested but not found! Disabling Gflags support")
	UPDATE_CACHE_VARIABLE(SPC_GFLAGS OFF)
    endif(GFLAGS_FOUND)

else(SPC_GFLAGS) # gflags not requested.
    message("GFLAGS not enabled. All executables will be not build")
    UPDATE_CACHE_VARIABLE(SPC_BUILD_TOOLS OFF)
    UPDATE_CACHE_VARIABLE(SPC_BUILD_SANDBOX_TOOLS OFF)
    UPDATE_CACHE_VARIABLE(SPC_BUILD_EXAMPLES_TOOLS OFF)
    UPDATE_CACHE_VARIABLE(SPC_BUILD_TESTING_TOOLS OFF)
endif(SPC_GFLAGS)


message("Minilog flag: " ${SPC_MINILOG})

IF (SPC_MINIGLOG)
  MESSAGE("-- Compiling minimal glog substitute (the one from ceres-solver).")
  SET(GLOG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/miniglog)
  SET(GLOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/miniglog)

  MESSAGE("-- Using minimal glog substitute (include): ${GLOG_INCLUDE_DIRS}")

#    add_library(miniglog STATIC miniglog/glog/logging.cc)

    set(GLOG_LIBRARIES )
    set(GLOG_LIBRARY )

    MARK_AS_ADVANCED(FORCE GLOG_INCLUDE_DIR
			 GLOG_LIBRARY)



ELSE (SPC_MINIGLOG)

  # Don't search with REQUIRED so that configuration continues if not found and
  # we can output an error messages explaining MINIGLOG option.
  FIND_PACKAGE(Glog)
  IF (GLOG_FOUND)
    message("GLOG FOUND")
  ELSE (GLOG_FOUND)
    MESSAGE(FATAL_ERROR "Can't find Google Log. Please set GLOG_INCLUDE_DIR & "
      "GLOG_LIBRARY or enable MINIGLOG option to use minimal glog "
      "implementation.")
  ENDIF (GLOG_FOUND)

ENDIF (SPC_MINIGLOG)




    INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIRS})








message ("${SPC_DEFINITIONS}")
if (SPC_BUILD_WITH_GPROF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg" )
endif()


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




# The interesting stuff goes here
# ===============================

# Add all targets to the build-tree export set - also binaries target should be added
export(TARGETS ${SPC_LIBRARIES} FILE "${PROJECT_BINARY_DIR}/SPCTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE SPC)

#### BUILD TREE
## ... for the build tree - to be done!
set (CONF_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
set (CONF_CEREAL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/submodules/cereal/include")
set (CONF_LIBRARY_DIRS "${LIBRARY_OUTPUT_PATH}")
set (CONF_NANOFLANN_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/submodules/nanoflann/include")
set (CONF_MINIGLOG_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/miniglog")

configure_file(SPCConfig.cmake.in "${PROJECT_BINARY_DIR}/SPCConfig.cmake" @ONLY)

## the ConfigVersion.cmake
configure_file(SPCConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/SPCConfigVersion.cmake" @ONLY)

#### INSTALL TREE
if(SPC_ENABLE_INSTALL)
    ## reset conf_include_dirs for the install tree
    set (CONF_INCLUDE_DIRS "${SPC_INSTALL_INCLUDE_DIR}")
    set (CONF_CEREAL_INCLUDE_DIRS "${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty")
    set (CONF_NANOFLANN_INCLUDE_DIRS "${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty")
    set (CONF_LIBRARY_DIRS "${SPC_INSTALL_LIB_DIR}")
    set (CONF_MINIGLOG_INCLUDE_DIRS "${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty/miniglog")


    ## reconfigure the SPCConfig.cmake putting it in a subdirectory of the build tree
    configure_file(SPCConfig.cmake.in "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/SPCConfig.cmake" @ONLY)


    # Install the FooBarConfig.cmake and FooBarConfigVersion.cmake in the install tree
    install(FILES "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/SPCConfig.cmake"
                  "${PROJECT_BINARY_DIR}/SPCConfigVersion.cmake"
      DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

    ## Install the export set for use with the install-tree
    install(EXPORT SPCTargets DESTINATION
                   "${INSTALL_CMAKE_DIR}" COMPONENT dev)

    INSTALL( DIRECTORY miniglog DESTINATION "${SPC_INSTALL_INCLUDE_DIR}/spc/3rdParty" )
endif()

if(SPC_BUILD_TESTS)
#    add_subdirectory(submodules/googletest/googletest)
    add_subdirectory(tests)
endif()
