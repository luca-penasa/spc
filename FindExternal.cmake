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
    #    UPDATE_CACHE_VARIABLE(SPC_BUILD_TOOLS OFF)
    #    UPDATE_CACHE_VARIABLE(SPC_BUILD_SANDBOX_TOOLS OFF)
    #    UPDATE_CACHE_VARIABLE(SPC_BUILD_EXAMPLES_TOOLS OFF)
    #    UPDATE_CACHE_VARIABLE(SPC_BUILD_TESTING_TOOLS OFF)
endif(SPC_GFLAGS)

#### NANOFLANN
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/submodules/nanoflann/include/")

############ CEREAL ####################################
set(CEREAL_INCLUDE_DIRS "submodules/cereal/include")

include_directories(${CEREAL_INCLUDE_DIRS})

set( BOOL CEREAL_IS_INTERNAL TRUE)
set(CEREAL_IS_INTERNAL TRUE CACHE BOOL "if internal cereal is used")
######################################################



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



add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/submodules/aquila")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/submodules/aquila")

set(AQUILA_LIBRARIES Aquila Ooura_fft)

install(TARGETS ${AQUILA_LIRARIES} EXPORT Aquila RUNTIME DESTINATION "${SPC_INSTALL_LIB_DIR}" COMPONENT lib)

