cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(SPC)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER) #in lowercase

set(SPC_LIBRARIES CACHE INTERNAL "SPC libs" FORCE)

option(BUILD_SPC_SHARED "Build spc library as a shared lib." ON)
option(BUILD_TOOLS "Build the tools." ON)
option(BUILD_SANDBOX_TOOLS "Build sandbox tools." OFF)
option(BUILD_MICMAC_RELATED "Build MICMAC-related code." ON)
option(BUILD_PYTHON_WRAPPINGS "Build python interface for the library. Only some classes have been wrapped!" ON)
option(BUILD_SPC_DOC "Build SPC doc" OFF)
option(BUILD_TESTS "Do tests" OFF)
option(BUILD_QGEO_CC_PLUGIN "Build SPC-based plugin for CloudCompare" ON)
option(SWIG_WRAPPINGS "Use swigs to generate wrappings for some methods" ON)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#we always need eigen
#find_package(Eigen REQUIRED)
#include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
#add_definitions(-DEIGEN_USE_NEW_STDVECTOR
#                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

#PCL!
find_package(PCL 1.6 EXACT REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Standard set of flags for c++
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -std=c++11 -fopenmp -fPIC -lpthread -lgcc_s")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -std=c++0x -fopenmp -fPIC -lpthread")

#set(CMAKE_BUILD_TYPE DEBUG) #force it to debug for now

#add custom functions and macros
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake)

# includes starts from here
include_directories(${CMAKE_CURRENT_SOURCE_DIR})




#micmac lib part
if(BUILD_MICMAC_RELATED)
    add_subdirectory(external)
    set(CULTURE_3D_BUILD_DIR "/home/luca/Code/culture3d/build" CACHE STRING "path where a culture3d build has been compiled is located. This is my default. you should change to your needs")
    set(CULTURE_3D_SOURCE_DIR "/home/luca/Code/culture3d" CACHE STRING "path where culture 3d source resides. This is my default. Change o your needs")
    link_directories(${CULTURE_3D_BUILD_DIR}/src)
endif()

#qgeo plugin for cloud compare??
if(BUILD_QGEO_CC_PLUGIN)
    set(CC_BUILD_DIR "/home/luca/Code/CC_git_clean/build" CACHE STRING "Dir with CloudCompare build")
    set(CC_SOURCE_DIR "/home/luca/Code/CC_git_clean" CACHE STRING "Dir with CloudCompare souces")
    set(QGEO_INSTALL_DIR "/home/luca/apps/bin/plugins" CACHE STRING "Where to intall the qGEO plugin")

    add_definitions(-DQGEO)

    find_package(Qt4 4 COMPONENTS REQUIRED)
    set(QT_USE_QTOPENGL TRUE)
    include(${QT_USE_FILE})
    add_definitions(${QT_DEFINITIONS})
    include_directories(${QT_QTOPENGL_INCLUDE_DIR})



    #we need qwt ############
    find_path( QWT_INCLUDE_DIR qwt_plot.h /usr/include/qwt6 )
    find_library( QWT_LIBRARIES NAMES qwt6) #explicit look for qwt6

    include( FindPackageHandleStandardArgs )
    find_package_handle_standard_args( QWT DEFAULT_MSG QWT_LIBRARIES QWT_INCLUDE_DIR )

    if(QWT_FOUND)
      mark_as_advanced(QWT_LIBRARIES QWT_INCLUDE_DIR )
      include_directories(${QWT_INCLUDE_DIR})
    endif()


    include_directories(${CC_SOURCE_DIR}/libs/qCC_db)
    include_directories(${CC_SOURCE_DIR}/CC/include)

    find_package(Qt4 REQUIRED)
    include(${QT_USE_FILE})
    add_definitions(${QT_DEFINITIONS})

endif()



# the main libraries
add_subdirectory(spc)

if(BUILD_QGEO_CC_PLUGIN) ##qGEO will use various things defined after spc
    add_subdirectory(qGEO)    
endif()

#also tools if needed
if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

#also tools if needed
if(BUILD_SANDBOX_TOOLS)
    add_subdirectory(sandbox)
endif()

#add_subdirectory(sandbox)

