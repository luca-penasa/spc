include(ExternalProject)




ExternalProject_Add(googletest
#    PREFIX "external_projects"
    GIT_REPOSITORY git@github.com:google/googletest.git
    GIT_TAG release-1.7.0
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/external
#    DOWNLOAD_DIR "${CMAKE_BINARY_DIR}"

#    CONFIGURE_COMMAND "cd ${CMAKE_BINARY_DIR}/googletest.git/googletest/ && mkdir -p build && cd build && cmake ../"

#    PATCH_COMMAND ${CMAKE_COMMAND} -E copy_if_different  googletest.git/googletest ${CMAKE_BINARY_DIR}

#    SOURCE_DIR "${CMAKE_BINARY_DIR}/googletest"
##    BINARY_DIR "${CMAKE_BINARY_DIR}/googletest-build"
    INSTALL_COMMAND   ""
    )

SET(GTEST_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/external/src/googletest/include)
SET(GTEST_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/external/src/googletest-build)
set(GTEST_LIBRARIES gtest_main gtest)

enable_testing()
#find_package(GTest REQUIRED)





#include_directories()

#include_directories(${GTEST_INCLUDE_DIRS})


#set(GTEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../submodules/googletest/googletest")

#include_directories(${GTEST_DIR})
message(${GTEST_LIB_DIR})
include_directories(${GTEST_INCLUDE_DIR})
link_directories(${GTEST_LIB_DIR})
#add_library(gtest-all STATIC ${GTEST_DIR}/src/gtest-all.cc)
#add_library(gtest-main STATIC ${GTEST_DIR}/src/gtest_main.cc)

#add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND}
#		  DEPENDS elements)

add_executable(test_elements elements.cpp)
target_link_libraries(test_elements ${GTEST_LIBRARIES} ${SPC_LIBRARIES})

add_test(AllTestsInElements test_elements)

