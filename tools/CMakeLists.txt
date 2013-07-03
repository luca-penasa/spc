set(MY_TOOLS_PREFIX "spc_")

if(BUILD_MICMAC_RELATED)
    link_directories(${CULTURE_3D_BUILD_DIR}/src)
endif()



find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

include_directories(/usr/include/eigen3/)
# include_directories(${LIDARLIB_SOURCE_DIR})

spc_add_executable(${MY_TOOLS_PREFIX}bilateral bilateral.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}bilateral ${PCL_LIBRARIES})

#spc_add_executable(align_icp align_ICP.cpp)
#target_link_libraries(align_icp ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#spc_add_executable(align_icp2 align_ICP_old.cpp)
#target_link_libraries(align_icp2 ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})
#
# spc_add_executable(pointer_test pointer_test.cpp)
#
# spc_add_executable(generate_example_cloud generate_cloud.cpp)
# target_link_libraries(generate_example_cloud ${PCL_LIBRARIES})
#
# spc_add_executable(convert_cloud convert_cloud.cpp)
# target_link_libraries(convert_cloud ${PCL_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}xyz2pcd xyz2pcd.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}xyz2pcd ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#spc_add_executable(compute_features compute_features.cpp)
#target_link_libraries(compute_features ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}compute_stats compute_stats.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_stats ${PCL_LIBRARIES} )

spc_add_executable(${MY_TOOLS_PREFIX}compute_intensity_spin compute_intensity_spin.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_intensity_spin ${PCL_LIBRARIES} )

spc_add_executable(${MY_TOOLS_PREFIX}compute_sensor_distances compute_distances.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_sensor_distances ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${SPC_LIBRARIES})

# spc_add_executable(try_kd_tree try_kd_tree.cpp)
# target_link_libraries(try_kd_tree ${PCL_LIBRARIES} ${SPC_LIBRARIES})


# spc_add_executable(test_ks test_ks.cpp)
# target_link_libraries(test_ks ${PCL_LIBRARIES} ${SPC_LIBRARIES})


spc_add_executable(${MY_TOOLS_PREFIX}distance_detrend distance_detrend.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}distance_detrend ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}ptx2pcd ptx2pcd.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}ptx2pcd ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#spc_add_executable(resample_polynomial resample_polynomial.cpp)
#target_link_libraries(resample_polynomial ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#spc_add_executable(initial_align initial_align.cpp initial_align.h)
#target_link_libraries(initial_align ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#disabled for now -> it need pcl compiled with the segmentation module activated
#spc_add_executable(${MY_TOOLS_PREFIX}euclidean_cluster euclidean_cluster.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}euclidean_cluster ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})


spc_add_executable(${MY_TOOLS_PREFIX}compute_presence_curve compute_presence_curve.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_presence_curve ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}extract_pois extract_pois.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}extract_pois ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}compute_normals_eigenvalues compute_normals_eigenvalues.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_normals_eigenvalues ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}estimate_correction_curve estimate_correction_curve.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}estimate_correction_curve ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}apply_correction_curve apply_correction_curve.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}apply_correction_curve ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})



spc_add_executable(${MY_TOOLS_PREFIX}merge_clouds_with_config merge_clouds_with_config.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}merge_clouds_with_config ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})



spc_add_executable(${MY_TOOLS_PREFIX}produce_fields_list produce_fields_list.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}produce_fields_list ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}produce_training_data produce_training_data.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}produce_training_data ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}convert_pcd_to_svm_format convert_pcd_to_svm_format.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}convert_pcd_to_svm_format ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})


spc_add_executable(${MY_TOOLS_PREFIX}merge_classification_results merge_classification_results.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}merge_classification_results ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}subset_from_geometry subset_from_geometry.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}subset_from_geometry ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})


