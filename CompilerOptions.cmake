
if(UNIX)
    list(APPEND SPC_COMPILER_OPTIONS -fPIC)
endif()

if(WIN32)
#    add_definitions( -DSPC_LIB_EXPORTS )
    list(APPEND SPC_COMPILER_DEFINITIONS CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE)
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


    list(APPEND SPC_COMPILER_DEFINITIONS LIBCXX_CXX_ABI=libstdc++)
endif()


########################### COMPILER FLAGS ##################################
if(MSVC)
    list(APPEND SPC_COMPILER_OPTIONS /MP /bigobj)
elseif (UNIX)
        list(APPEND SPC_COMPILER_OPTIONS  -Wno-narrowing )
endif()



if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  list(APPEND SPC_COMPILER_DEFINITIONS BOOST_DISABLE_ASSERTS EIGEN_NO_DEBUG)
endif()


option(SPC_BUILD_WITH_GPROF "Build with -pg flags for profiling." OFF)

if (SPC_BUILD_WITH_GPROF)
    list(APPEND SPC_COMPILER_OPTIONS -pg)
endif()





