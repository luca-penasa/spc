cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(spc)


set(CMAKE_VERBOSE_MAKEFILE on)

# these variable will be populated by next macros in includes
set(SPC_LINK_TARGETS)
set(SPC_SUBMODULES_INCLUDES)
set(SPC_COMPILER_DEFINITIONS)
set(SPC_COMPILER_OPTIONS)
set(SPC_COMPILER_FEATURES)

include(FindExternal.cmake)
include(CompilerOptions.cmake)
include(Versioning.cmake)


################################## INSTALL OPTIONS #####################################################
set(SPC_INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(SPC_INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(SPC_INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
######################################################################################

INCLUDE(cmake/UpdateCacheVariable.cmake) #courtesy of ceres-solver

#my definitions

# my libs
set(SPC_MODULES CACHE INTERNAL "SPC libs" FORCE) #this will collect all the modular libs spc will create

# all libraries into a single dir for the build tree
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_subdirectory(submodules)



############################# OPTIONS ######################################################################
option(SPC_BUILD_SHARED "Build spc library as a shared lib." ON)

#option(SPC_BUILD_SANDBOX_TOOLS "Build sandbox tools." OFF)
option(SPC_BUILD_EXAMPLES_TOOLS "Build examples tools." OFF)

option(SPC_USE_IWYU "Use include-what-you-use to notify about unused includes" OFF)

#OPTION(SPC_GFLAGS "Enable Google Flags. Needed by the tools" OFF)
option(SPC_BUILD_TESTS "Do tests" OFF)
option(SPC_ENABLE_INSTALL "Enable/Disable the install target" OFF)

option(SPC_WITH_PCL "Compile SPC lib with PCL support - Many more dependencies" OFF)


include(CMakeDependentOption)

cmake_dependent_option(SPC_BUILD_TOOLS "Build the tools." ON
                       "SPC_WITH_PCL" OFF)
#option(SPC_GFLAGS_IN_NAMESPACE_GOOGLE "says if gflags is expected to be into the google namespace" OFF)





#add custom functions and macros
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake)






#micmac lib part
if(SPC_BUILD_MICMAC_RELATED)
    set(CULTURE_3D_BUILD_DIR "/home/luca/Code/culture3d/build" CACHE STRING "path where a culture3d build has been compiled is located. This is my default. you should change to your needs")
    set(CULTURE_3D_SOURCE_DIR "/home/luca/Code/culture3d" CACHE STRING "path where culture 3d source resides. This is my default. Change o your needs")
    link_directories(${CULTURE_3D_BUILD_DIR}/src)
endif()



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


if(SPC_USE_IWYU)
    find_program(iwyu_path NAMES include-what-you-use iwyu)
    if(NOT iwyu_path)
        message(FATAL_ERROR "Could not find the program include-what-you-use")
    endif()
endif()

OPTION(SPC_OSGTEST "Compile test application for point cloud visualization" OFF)
if(SPC_OSGTEST)
    add_subdirectory(osg_test)
endif()

INCLUDE(PackageExport.cmake)


# include CMakePackageConfigHelpers macro
#include(CMakePackageConfigHelpers)

# set version
#set(version 0.0.1)

## generate the version file for the config file
#write_basic_package_version_file(
#  "${CMAKE_CURRENT_BINARY_DIR}/SPCConfigVersion.cmake"
#  VERSION "${version}"
#  COMPATIBILITY AnyNewerVersion
#)

## create config file
#configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
#  "${CMAKE_CURRENT_BINARY_DIR}/SPCConfig.cmake"
#  INSTALL_DESTINATION lib/cmake/SPC
#  NO_CHECK_REQUIRED_COMPONENTS_MACRO
#)

## install config files
#install(FILES
#          "${CMAKE_CURRENT_BINARY_DIR}/SPCConfig.cmake"
#          "${CMAKE_CURRENT_BINARY_DIR}/SPCConfigVersion.cmake"
#        DESTINATION lib/cmake/MathFunctions
#)
