set(MY_TOOLS_PREFIX "spc_")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../)

if(BUILD_MICMAC_RELATED)
    link_directories(${CULTURE_3D_BUILD_DIR}/src)
endif()

set(CMAKE_VERBOSE_MAKEFILE OFF)

add_definitions(${SPC_DEFINITIONS})

message("the defs for tools: ${SPC_DEFINITONS}")

#    FIND_PACKAGE( OpenMP REQUIRED)
#    if(OPENMP_FOUND)
#        message("OPENMP FOUND")
#        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#        set(SPC_DEFINITIONS ${SPC_DEFINITIONS} "-DUSE_OPENMP")
##        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -lpthread -lgcc_s" )
#    endif()

#if (UNIX) #this maybe is not needed at all...
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
#endif()

#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

#find_package(VTK REQUIRED)
#include_directories(${VTK_INCLUDE_DIRS})

#find_package(SPC REQUIRED)
#include_directories(${SPC_INCLUDE_DIRS})

#include_directories(/usr/include/eigen3/)
# include_directories(${LIDARLIB_SOURCE_DIR})

spc_add_executable(${MY_TOOLS_PREFIX}bilateral spc_bilateral.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}bilateral ${PCL_LIBRARIES})

#spc_add_executable(align_icp align_ICP.cpp)
#target_link_libraries(align_icp ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})



spc_add_executable(${MY_TOOLS_PREFIX}test_tool spc_test_tool.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}test_tool ${PCL_LIBRARIES} ${SPC_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}extract_selection_example spc_extract_selection_example.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}extract_selection_example ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})



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

spc_add_executable(${MY_TOOLS_PREFIX}xyz2pcd spc_xyz2pcd.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}xyz2pcd ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#spc_add_executable(compute_features compute_features.cpp)
#target_link_libraries(compute_features ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}compute_stats spc_compute_stats.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_stats ${PCL_LIBRARIES} )

spc_add_executable(${MY_TOOLS_PREFIX}compute_intensity_spin spc_compute_intensity_spin.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_intensity_spin ${PCL_LIBRARIES} )

#spc_add_executable(${MY_TOOLS_PREFIX}compute_sensor_distances spc_compute_distances.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}compute_sensor_distances ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${SPC_LIBRARIES})

# spc_add_executable(try_kd_tree try_kd_tree.cpp)
# target_link_libraries(try_kd_tree ${PCL_LIBRARIES} ${SPC_LIBRARIES})


# spc_add_executable(test_ks test_ks.cpp)
# target_link_libraries(test_ks ${PCL_LIBRARIES} ${SPC_LIBRARIES})


#spc_add_executable(${MY_TOOLS_PREFIX}distance_detrend distance_detrend.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}distance_detrend ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}ptx2pcd spc_ptx2pcd.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}ptx2pcd ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#spc_add_executable(resample_polynomial resample_polynomial.cpp)
#target_link_libraries(resample_polynomial ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#spc_add_executable(${MY_TOOLS_PREFIX}initial_align spc_initial_align.cpp spc_initial_align.h)
#target_link_libraries(${MY_TOOLS_PREFIX}initial_align ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#disabled for now -> it need pcl compiled with the segmentation module activated
#spc_add_executable(${MY_TOOLS_PREFIX}euclidean_cluster euclidean_cluster.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}euclidean_cluster ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})


spc_add_executable(${MY_TOOLS_PREFIX}compute_presence_curve spc_compute_presence_curve.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_presence_curve ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}extract_pois spc_extract_pois.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}extract_pois ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}compute_normals_eigenvalues spc_compute_normals_eigenvalues.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_normals_eigenvalues ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#spc_add_executable(${MY_TOOLS_PREFIX}estimate_correction_curve estimate_correction_curve.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}estimate_correction_curve pcl_search ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

#spc_add_executable(${MY_TOOLS_PREFIX}apply_correction_curve apply_correction_curve.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}apply_correction_curve pcl_search ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})



spc_add_executable(${MY_TOOLS_PREFIX}merge_clouds_with_config spc_merge_clouds_with_config.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}merge_clouds_with_config ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})



spc_add_executable(${MY_TOOLS_PREFIX}produce_fields_list spc_produce_fields_list.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}produce_fields_list ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}produce_training_data spc_produce_training_data.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}produce_training_data ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}convert_pcd_to_svm_format spc_convert_pcd_to_svm_format.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}convert_pcd_to_svm_format ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})


spc_add_executable(${MY_TOOLS_PREFIX}merge_classification_results spc_merge_classification_results.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}merge_classification_results ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

spc_add_executable(${MY_TOOLS_PREFIX}subset_from_geometry spc_subset_from_geometry.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}subset_from_geometry ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})


#spc_add_executable(${MY_TOOLS_PREFIX}get_mean_std_sampling_step spc_get_mean_std_sampling_step.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}get_mean_std_sampling_step ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})


spc_add_executable(${MY_TOOLS_PREFIX}append_orientations_to_pcd spc_append_orientations_to_pcd)
target_link_libraries(${MY_TOOLS_PREFIX}append_orientations_to_pcd ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})

message(" glfags libs::::: " ${GFLAGS_LIBRARIES}})
spc_add_executable(${MY_TOOLS_PREFIX}compute_calibration_dataset spc_compute_calibration_dataset)
target_link_libraries(${MY_TOOLS_PREFIX}compute_calibration_dataset ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})



spc_add_executable(${MY_TOOLS_PREFIX}compute_intensity_calibration spc_compute_intensity_calibration.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_intensity_calibration ${PCL_LIBRARIES} ${SPC_LIBRARIES} ${VTK_LIBRARIES})





#FIND_PACKAGE(Ceres REQUIRED)
#INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})


#spc_add_executable(${MY_TOOLS_PREFIX}calibration_testing spc_calibration_testing.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}calibration_testing ${SPC_LIBRARIES}  ${CERES_LIBRARIES})

#spc_add_executable(${MY_TOOLS_PREFIX}calibration_testing2 spc_calibration_testing2.cpp)
#target_link_libraries(${MY_TOOLS_PREFIX}calibration_testing2 ${SPC_LIBRARIES}  ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES})


spc_add_executable(${MY_TOOLS_PREFIX}calibrate_rbf spc_calibrate_rbf)
target_link_libraries(${MY_TOOLS_PREFIX}calibrate_rbf ${SPC_LIBRARIES} spc_methods ${GFLAGS_LIBRARIES})


spc_add_executable(${MY_TOOLS_PREFIX}xml2spc spc_xml2spc.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}xml2spc ${SPC_LIBRARIES})


#spc_add_executable(${MY_TOOLS_PREFIX} apply_rbf_model apply_rbf_model.cpp)
#target_link_libraries(apply_rbf_model ${SPC_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})



spc_add_executable(${MY_TOOLS_PREFIX}apply_rbf_model spc_apply_rbf_model.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}apply_rbf_model ${SPC_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})



spc_add_executable(${MY_TOOLS_PREFIX}transfer_fields spc_transfer_fields.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}transfer_fields ${SPC_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})


spc_add_executable(${MY_TOOLS_PREFIX}compute_angles_distances spc_compute_angles_distances.cpp)
target_link_libraries(${MY_TOOLS_PREFIX}compute_angles_distances ${SPC_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})


spc_add_executable(spc_quad_example spc_quad_example.cpp)
target_link_libraries(spc_quad_example)


spc_add_executable(${MY_TOOLS_PREFIX}calibrate_intensity_rbf spc_calibrate_intensity_rbf)
target_link_libraries(${MY_TOOLS_PREFIX}calibrate_intensity_rbf ${SPC_LIBRARIES} spc_methods ${GFLAGS_LIBRARIES})

