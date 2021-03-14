

list(APPEND SPC_COMPILER_DEFINITIONS ELPP_THREAD_SAFE)
list(APPEND SPC_COMPILER_FEATURES cxx_std_14)


if (SPC_WITH_PCL)
    find_package(PCL 1.11)
#    add_compile_definitions(SPC_WITH_PCL)
    list(APPEND SPC_COMPILER_DEFINITIONS SPC_WITH_PCL)
    list(APPEND SPC_LINK_TARGETS ${PCL_LIBRARIES})
else()
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
    list(APPEND SPC_LINK_TARGETS Eigen3::Eigen)


    if(WIN32)
        # also boost! please use static libs as def on win so that we dont have to move boost libs along with the app
        set(Boost_USE_STATIC_LIBS   ON)
    endif()

    set(Boost_USE_MULTITHREADED ON)

    find_package(Boost REQUIRED filesystem regex)

    list(APPEND SPC_LINK_TARGETS ${Boost_LIBRARIES})
endif()


# always use openmp if possible
FIND_PACKAGE( OpenMP)
if(OPENMP_FOUND)
    list(APPEND SPC_LINK_TARGETS OpenMP::OpenMP_CXX)
endif()


# gflags needed only if tools are enabled
if(SPC_BUILD_TOOLS)
    find_package(gflags REQUIRED)
    list(APPEND SPC_LINK_TARGETS gflags)
endif()


#SUBMODULES path , to simplify call below
SET(SP "${CMAKE_SOURCE_DIR}/submodules")

# and their paths
list(APPEND SPC_SUBMODULES_INCLUDES "${SP}/nanoflann/include"
    "${SP}/cereal/include"
    "${SP}/easyloggingpp/src")


