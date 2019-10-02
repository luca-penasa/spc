set(MY_TOOLS_PREFIX "sb_")

if(BUILD_MICMAC_RELATED)
    link_directories(${CULTURE_3D_BUILD_DIR}/src)
endif()



find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

include_directories(/usr/include/eigen3/)
# include_directories(${LIDARLIB_SOURCE_DIR})

find_package( OpenCV REQUIRED )


spc_add_executable(${MY_TOOLS_PREFIX}match_keypoints mm_match_keypoints.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}match_keypoints ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})


set(files  mm_keypoints_to_txt.cpp fileIO.h fileIO.cpp)
spc_add_executable(${MY_TOOLS_PREFIX}keypoints_to_txt "${files}")
target_link_libraries(${MY_TOOLS_PREFIX}keypoints_to_txt ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES} flann)

spc_add_executable(${MY_TOOLS_PREFIX}test test.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}test ${OpenCV_LIBS})



set(files  flannMatcher.cpp fileIO.h fileIO.cpp)
spc_add_executable(${MY_TOOLS_PREFIX}flannMatcher "${files}")
target_link_libraries(${MY_TOOLS_PREFIX}flannMatcher flann)

set(files  mm_read_keypoints.cpp)
spc_add_executable(${MY_TOOLS_PREFIX}read_keypoints "${files}")
target_link_libraries(${MY_TOOLS_PREFIX}read_keypoints ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES} flann)


#spc_add_executable(${MY_TOOLS_PREFIX}example_nurbs_fitting_closed_curve example_nurbs_fitting_closed_curve.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}example_nurbs_fitting_closed_curve  ${PCL_LIBRARIES})

#spc_add_executable(${MY_TOOLS_PREFIX}example_nurbs_fitting_curve2d example_nurbs_fitting_curve2d.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}example_nurbs_fitting_curve2d  ${PCL_LIBRARIES})


#spc_add_executable(${MY_TOOLS_PREFIX}example_nurbs_fitting_surface example_nurbs_fitting_surface.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}example_nurbs_fitting_surface  ${PCL_LIBRARIES})


spc_add_executable(${MY_TOOLS_PREFIX}compute_keypoints compute_keypoints.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_keypoints  ${SPC_LIBRARIES})


#add_subdirectory(ties)
