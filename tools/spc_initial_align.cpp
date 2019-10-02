#include "spc_initial_align.h"
#include "strings.h"


int 
saveKeepingFields(const pcl::PointCloud<pcl::PointXYZ> &transformed_cloud, 
									const std::string infilename,
									const std::string outfilename
 								)

{
	
	// convert transformed_cloud to ros msgs
	sensor_msgs::PointCloud2::Ptr output2 (new sensor_msgs::PointCloud2);
	toROSMsg(transformed_cloud, *output2);
	
	//reload input cloud
	sensor_msgs::PointCloud2::Ptr original (new sensor_msgs::PointCloud2);
	pcl::io::loadPCDFile (infilename.c_str(), *original);
	
	sensor_msgs::PointCloud2::Ptr all (new sensor_msgs::PointCloud2);
	
	//concatenate the two
	pcl::concatenateFields(*original,*output2, *all);
	
	pcl::io::savePCDFile (outfilename.c_str(), *all, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
}

int 
main(int argc, char* argv[])
{
	all_settings settings;
	if (settings.parse(argc, argv) == -1)
	{
		return -1;
	}
	settings.print();
	
	//load the clouds as sensor_msgs, we will downcast the clouds later to a fixed type
	sensor_msgs::PointCloud2::Ptr ros_target_cloud_ptr (new sensor_msgs::PointCloud2);
	sensor_msgs::PointCloud2::Ptr ros_source_cloud_ptr (new sensor_msgs::PointCloud2);
	
	pcl::io::loadPCDFile(settings.target_filename.c_str(), *ros_target_cloud_ptr);
	pcl::io::loadPCDFile(settings.source_filename.c_str(), *ros_source_cloud_ptr);
	
	//create the clouds with intensities
	pcl::PointCloud<PointIntT>::Ptr target_cloud_ptr (new pcl::PointCloud<PointIntT>);
	pcl::PointCloud<PointIntT>::Ptr source_cloud_ptr (new pcl::PointCloud<PointIntT>);
	
	//get the method to be used
	string method_to_use = settings.downsampling_method;
	
	if (method_to_use == "sift")
	{
		//check if we have an intensity field for both clouds
		int target_have_int = pcl::getFieldIndex(*ros_target_cloud_ptr, "intensity");
		int source_have_int = pcl::getFieldIndex(*ros_source_cloud_ptr, "intensity");
		if (target_have_int == -1)
		{
			pcl::console::print_error ("Target cloud %s does not have intensity field. --sift method cannot be used.\n", settings.target_filename.c_str());
			return -1;
		}
		else if (source_have_int == -1)
		{
			pcl::console::print_error ("Source cloud %s does not have intensity field. --sift method cannot be used.\n", settings.source_filename.c_str());
			return -1;
		}
		
		
	}

	

	pcl::fromROSMsg(*ros_target_cloud_ptr, *target_cloud_ptr);
	pcl::fromROSMsg(*ros_source_cloud_ptr, *source_cloud_ptr);
	
	//two clouds for storing keypoints
	pcl::PointCloud<KeysT>::Ptr target_keypoints_ptr (new pcl::PointCloud<KeysT>);
	pcl::PointCloud<KeysT>::Ptr source_keypoints_ptr (new pcl::PointCloud<KeysT>);
	
	if (method_to_use == "sift") //xyz points are returned (KeysT)
	{
		//Compute keypoints - they always are simple xyz points
		compute_sift<PointIntT, PointT>(target_cloud_ptr, target_keypoints_ptr, settings.sift);
		compute_sift<PointIntT, PointT>(source_cloud_ptr, source_keypoints_ptr, settings.sift);
	}
	
	
	else if (method_to_use == "voxelgrid") // an xyz type is always returned!
	{
		//simply downsample the cloud
		compute_voxelgrid<PointIntT, KeysT>(target_cloud_ptr, target_keypoints_ptr, settings.voxelgrid);
		compute_voxelgrid<PointIntT, KeysT>(source_cloud_ptr, source_keypoints_ptr, settings.voxelgrid);
		
	}
	
	else if (method_to_use == "harris") //also this method makes use of intensity
	{
		print_error("Harris method not implemented yet.\n");
		return -1;
		compute_harris<PointIntT, KeysT>(target_cloud_ptr, target_keypoints_ptr, settings.harris);
		compute_harris<PointIntT, KeysT>(source_cloud_ptr, source_keypoints_ptr, settings.harris);
	}
	
	
	//Now compute normals for each of the point in the keypoint dataset.
	pcl::PointCloud<pcl::Normal>::Ptr target_normals_ptr (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr source_normals_ptr (new pcl::PointCloud<pcl::Normal>);
	
	
	if (method_to_use == "voxelgrid")
	{
		//compute - - for ALL the input points
		compute_normals<PointT, PointT>(target_keypoints_ptr, target_keypoints_ptr, target_normals_ptr, settings.normals);
		compute_normals<PointT, PointT>(source_keypoints_ptr, source_keypoints_ptr, source_normals_ptr, settings.normals);
	}
	else
	{
		//compute - - for ALL the input points
		compute_normals<PointIntT, PointIntT>(target_cloud_ptr, target_cloud_ptr, target_normals_ptr, settings.normals);
		compute_normals<PointIntT, PointIntT>(source_cloud_ptr, source_cloud_ptr, source_normals_ptr, settings.normals);
	}

	//now for each point in keypoints compute its feature descriptor.
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_FPFH_ptr (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_FPFH_ptr (new pcl::PointCloud<pcl::FPFHSignature33>);
	
	if (method_to_use == "voxelgrid")
	{
		compute_FPFH<PointT, PointT>(target_keypoints_ptr, target_keypoints_ptr, target_normals_ptr, target_FPFH_ptr, settings.FPFHE);
		compute_FPFH<PointT, PointT>(source_keypoints_ptr, source_keypoints_ptr, source_normals_ptr, source_FPFH_ptr, settings.FPFHE);
	}
	else
	{
		compute_FPFH<PointT, PointIntT>(target_keypoints_ptr, target_cloud_ptr, target_normals_ptr, target_FPFH_ptr, settings.FPFHE);
		compute_FPFH<PointT, PointIntT>(source_keypoints_ptr, source_cloud_ptr, source_normals_ptr, source_FPFH_ptr, settings.FPFHE);
	}
	//Now start inital_align
	sac_ia_results sac_results;
	compute_sac_ia<PointT, pcl::FPFHSignature33>(target_keypoints_ptr,
								source_keypoints_ptr,
								target_FPFH_ptr,
								source_FPFH_ptr,
								settings.sac_ia,
								sac_results);
	
	sac_results.print();
	
	
	pcl::PointCloud<PointIntT> transformed_cloud;
	pcl::transformPointCloud (*source_cloud_ptr, transformed_cloud, sac_results.final_transformation);
	
	//we just need xyz data
	pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
	pcl::copyPointCloud<PointIntT, pcl::PointXYZ>(transformed_cloud, xyz_cloud);
	
	
	std::string infilename = settings.source_filename;
	std::string outfilename = stripExtension(infilename) + "_initally_aligned.pcd";
	
	saveKeepingFields(xyz_cloud, infilename, outfilename);
// 	pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud);
	
	
// 	
// 	
// 	///////////////////////////////////////////////
// 	////////     STUFF FOR VISUALIZATION   ////////
// 	///////////////////////////////////////////////
// // 	//Create a viewer and put the first cloud
// // 	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
// // 	viewer.setBackgroundColor (1, 1, 1);
// // 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_handler (cloud1, 0, 0, 0);
// // 	viewer.addPointCloud (cloud1, color_handler, "cloud1");
// // 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
// // 	viewer.initCameraParameters ();
// // 	viewer.addCoordinateSystem (1.0);
// 	
// 
// 	
// // 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoints_color_handler (keypoints1, 0, 255, 0);
// // 	viewer.addPointCloud<pcl::PointXYZI> (keypoints1, keypoints_color_handler, "keypoints");
// // 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
// 	
// 	
// // 	while (!viewer.wasStopped ())
// // 	{
// // 		viewer.spinOnce (100);
// // 		pcl_sleep(0.01);
// // 	}
// 	
	return 1;
}
