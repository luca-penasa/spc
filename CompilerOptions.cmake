
if(UNIX)
    add_definitions(-fPIC)
endif()

if(WIN32)
#    add_definitions( -DSPC_LIB_EXPORTS )
    add_definitions(-DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE)
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


    add_definitions(-DLIBCXX_CXX_ABI=libstdc++)
endif()


########################### COMPILER FLAGS ##################################
if (MINGW)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

elseif (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /bigobj" )
elseif (UNIX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing " )
endif()

# we need c++11


if(MINGW)
	add_definitions(-std=gnu++11)
else()
	add_definitions(-std=c++11)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()


option(SPC_BUILD_WITH_GPROF "Build with -pg flags for profiling." OFF)

if (SPC_BUILD_WITH_GPROF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg" )
endif()





