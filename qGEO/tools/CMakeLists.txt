cmake_minimum_required(VERSION 2.8)
project(tools)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#include_directories(${pcl_geology_SOURCE_DIR}/include)

include_directories( ${CC_DLL_SOURCE_DIR}/include )
include_directories( ${qCC_SOURCE_DIR} )
include_directories( ${qCC_SOURCE_DIR}/db_tree )
include_directories( ${qCC_BINARY_DIR} )
include_directories( ${QCC_DB_DLL_SOURCE_DIR} )


include_directories(${CMAKE_CURRENT_BINARY_DIR})




#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


#include_directories( ${Qwt6_INCLUDE_DIR} )



#add_executable(test test.cpp )
#target_link_libraries(test_cloud2image  QCC_DB_DLL CC_DLL geoutils ${VTK_LIBRARIES})
