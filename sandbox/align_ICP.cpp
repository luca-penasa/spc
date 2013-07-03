#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <boost/make_shared.hpp>
#include <pcl/registration/icp.h>
// #include <pcl/registration/impl/icp.hpp>

#include <pcl/registration/icp_nl.h>
#include <pcl/registration/impl/icp_nl.hpp>
#include <pcl/registration/transforms.h>

#include <pcl/keypoints/uniform_sampling.h>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <pcl/visualization/registration_visualizer.h>

#include "strings.h"

#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>


#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d_omp.hpp>




#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/impl/transformation_estimation_point_to_plane_lls.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>


struct PointXYZNormal
{
	PCL_ADD_POINT4D;
	PCL_ADD_NORMAL4D;
	float curvature;
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT   (PointXYZNormal,
																		 (float, x, x)
																		 (float, y, y)
																		 (float, z, z)
																		 (float, normal_x, normal_x)
																		 (float, normal_y, normal_y)
																		 (float, normal_z, normal_z)
																		 (float, curvature, curvature)
			   
)


class MyPointRepresentation : public pcl::PointRepresentation <PointXYZNormal>
{
	using pcl::PointRepresentation<PointXYZNormal>::nr_dimensions_;
public:
	MyPointRepresentation ()
	{
		// Define the number of dimensions
		nr_dimensions_ = 3;
		//nr_dimensions_ = 3;
	}
	
	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray (const PointXYZNormal &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		//     out[3] = p.curvature;
		//out[4] = p.intensity;
	}
};

using namespace pcl::console;
using namespace pcl::registration;

//Some typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> Cloud;
// typedef pcl::PointNormal PointNormalT;
// typedef pcl::PointCloud<PointNormalT> CloudWithNormals;
// typedef pcl::PointXYZINormal PointFull;
// typedef pcl::PointCloud<PointFull> CloudFull;


//icp-specfic
int max_itern_default = 50;
double max_correspondence_distance_default = 0.05;
double transformation_epsilon_defualt = 1e-3;
double ransac_threshold_default = 0.05;
int ransac_iterations_default = 5;
double euclidean_fitness_default = 1.0;

//for normal estimation
int knn_default = 20;

//initial downsampling
double voxelsize_default= 0.02;

// class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointXYZ>
// {
//   using pcl::PointRepresentation<pcl::PointXYZ>::nr_dimensions_;
// public:
//   MyPointRepresentation ()
//   {
//     // Define the number of dimensions
//     nr_dimensions_ = 3;
//     //nr_dimensions_ = 3;
//   }
// 
//   // Override the copyToFloatArray method to define our feature vector
//   virtual void copyToFloatArray (const PointFull &p, float * out) const
//   {
//     // < x, y, z, curvature >
//     out[0] = p.x;
//     out[1] = p.y;
//     out[2] = p.z;
// //     out[3] = p.curvature;
//     //out[4] = p.intensity;
//   }
// };


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
	pcl::concatenateFields(*original, *output2, *all);
	
	pcl::io::savePCDFile (outfilename.c_str(), *all, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
}



template <typename PointT> void
downsample(const typename pcl::PointCloud<PointT>::Ptr incloud, 
					 const double radius,
					 typename pcl::PointCloud<PointT>::Ptr outcloud
					)
{	
	pcl::PointCloud<int> ids;
	pcl::UniformSampling<PointT> uniform;
	uniform.setRadiusSearch (radius);  
	uniform.setInputCloud (incloud);
	uniform.compute (ids);
	pcl::copyPointCloud<PointT, PointT>(*incloud, ids.points, *outcloud);
	
}

void
compute_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, 
								const int knn,
								pcl::PointCloud<PointXYZNormal>::Ptr outcloud
)
{
	printf("Computing normals\n");
	//Compute normals and curvature
	
	pcl::NormalEstimationOMP<pcl::PointXYZ, PointXYZNormal> normal_estimator;
	pcl::PointCloud<PointXYZNormal>::Ptr cloud_with_normals (new pcl::PointCloud<PointXYZNormal>);
	
	normal_estimator.setKSearch (knn);
	normal_estimator.setInputCloud (incloud);
	normal_estimator.compute (*outcloud);

}

void
printHelp(int arg, char *argv[])
{
	print_info("Syntax is target.pcd source.pcd [params]\n");
	print_info("    -v       voxelsize for initial uniform sampling,   def %4.2f\n", voxelsize_default);
	print_info("    -mi      Maximum number of iterations,             def %i\n", max_itern_default);
	print_info("    -mcd     Maximum correspondence distance,          def %4.2f\n", max_correspondence_distance_default);
	print_info("    -te      Transoformation Epsilon,                  def %2.1E\n", transformation_epsilon_defualt);
	print_info("    -rt      RANSAC outlier rejection threshold,       def %4.2f\n", ransac_threshold_default);
	print_info("    -ri      number of RANSAC iterations,              def %i\n", ransac_iterations_default);
	print_info("    -ef      Euclidean Fitness Epsilon,                def %4.2f\n", euclidean_fitness_default);
	print_info("    -k       Number of points to use for normals,      def %i\n", knn_default);
	print_info("    -h       For this help\n");

}
int main(int argc, char *argv[])
{
	
	pcl::console::setVerbosityLevel(pcl::console::L_DEBUG); 
	//parse the input files
	std::vector<int> file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	if (file_indices.size() != 2)
	{
		print_error("You should give 2 point clouds. First target and then source.\n");
		printHelp(argc, argv);
		return -1;
	}
	
	//parse the commands
	double voxelsize = voxelsize_default;
	int max_itern = max_itern_default;
	double max_correspondence_distance = max_correspondence_distance_default;
	double transformation_epsilon = transformation_epsilon_defualt;
	double ransac_threshold = ransac_threshold_default;
	double euclidean_fitness = euclidean_fitness_default;
	int ransac_iterations = ransac_iterations_default;
	int knn = knn_default;
	
	parse_argument(argc, argv, "-v", voxelsize);
	parse_argument(argc, argv, "-mi", max_itern);
	parse_argument(argc, argv, "-mcd", max_correspondence_distance);
	parse_argument(argc, argv, "-te", transformation_epsilon);
	parse_argument(argc, argv, "-rt", ransac_threshold);
	parse_argument(argc, argv, "-ri", ransac_iterations);
	parse_argument(argc, argv, "-ef", euclidean_fitness);
	parse_argument(argc, argv, "-k", knn);
	
	
	if (find_switch(argc, argv, "-h"))
	{
		printHelp(argc, argv);
		return -1;
	}
	
	
	
	
	printf("Load Clouds\n");
	//Creating and loading clouds
	Cloud::Ptr cloud (new Cloud); //cloud to be transformed
	Cloud::Ptr tcloud (new Cloud); //target cloud
	

	//FILENAMES
	std::string tcloudname = argv[file_indices[0]]; 
	std::string cloudname = argv[file_indices[1]];
	
	//load clouds
	pcl::io::loadPCDFile(tcloudname.c_str(), *tcloud);
	pcl::io::loadPCDFile(cloudname.c_str(), *cloud);
	
	print_info("    -> Target cloud %s, with %i points.\n", tcloudname.c_str(), tcloud->points.size());
	print_info("    -> Source cloud %s, with %i points.\n", cloudname.c_str(), cloud->points.size());
	
	printf("Downsampling clouds\n");
	
	Cloud::Ptr cloud_down (new Cloud); //source downsampled
	Cloud::Ptr tcloud_down (new Cloud); // target downsampled
	
	downsample<PointT>(cloud, voxelsize, cloud_down);
	downsample<PointT>(tcloud, voxelsize, tcloud_down);
	
	print_info("    -> Now target cloud have %i point.\n", tcloud_down->points.size());
	print_info("    -> And source cloud have %i point.\n", cloud_down->points.size());
	
	
	/////////// COMPUTE NORMALS /////////////////
	pcl::PointCloud<PointXYZNormal>::Ptr cloud_with_normals (new pcl::PointCloud<PointXYZNormal>);
	pcl::PointCloud<PointXYZNormal>::Ptr tcloud_with_normals (new pcl::PointCloud<PointXYZNormal>);
	
	compute_normals(cloud_down, knn, cloud_with_normals);
	compute_normals(tcloud_down, knn, tcloud_with_normals);

	
// 	//save for debug
// 	pcl::io::savePCDFile("target_downsampled.pcd", *tcloud_down);
// 	pcl::io::savePCDFile("source_downsampled.pcd", *cloud_down);
	
// 	pcl::VoxelGrid<pcl::PointXYZI> vox_grid;
// 	vox_grid.setInputCloud (cloud);
// 	std::cout << cloud->points.size() << std::endl;
// 	vox_grid.setLeafSize (voxsize, voxsize, voxsize);
// 	vox_grid.setDownsampleAllData(1);
// 	vox_grid.filter(*cloud_down);
// 	std::cout << cloud_down->points.size() << std::endl;
// 	
// 	vox_grid.setInputCloud(tcloud);
// 	vox_grid.filter(*tcloud_down);
	
// 	if (0)
// 	{
// 		printf("Computing normals\n");
// 		//Compute normals and curvature
// 	// 	pcl::KdTreeFLANN<PointT>::Ptr kdtree (new pcl::KdTreeFLANN<PointT>);
// 		pcl::NormalEstimationOMP<PointT, PointNormalT> normal_estimator;
// 		CloudWithNormals::Ptr cloud_with_normals (new CloudWithNormals);
// 		CloudWithNormals::Ptr tcloud_with_normals (new CloudWithNormals);
// 		
// 		
// 	// 	normal_estimator.setSearchMethod (kdtree);
// 		//normal_estimator.setRadiusSearch (radius);
// 		normal_estimator.setKSearch (k_nr_points);
// 
// 
// 		normal_estimator.setInputCloud (cloud_down);
// 		normal_estimator.compute (*cloud_with_normals);
// 		
// 		CloudFull::Ptr cloudn (new CloudFull);
// 		pcl::concatenateFields (*cloud_with_normals,*cloud_down,  *cloudn);
// 	// 	normal_estimator.setRadiusSearch (radius);	
// 		normal_estimator.setInputCloud (tcloud_down);
// 		normal_estimator.compute (*tcloud_with_normals);
// 		
// 		CloudFull::Ptr tcloudn (new CloudFull);
// 		pcl::concatenateFields (*tcloud_with_normals,*tcloud_down,  *tcloudn);
// 		
// 		printf("Normals Computed!\n");
// 	}
  	MyPointRepresentation point_representation;
 	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z

 	float alpha[3] = {1, 1, 1};

  	point_representation.setRescaleValues (alpha); 
// 	pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal > reg;
	
		
		
		pcl::IterativeClosestPointNonLinear< PointXYZNormal, PointXYZNormal > reg;
		
	// Set the point representation
 	boost::shared_ptr<const MyPointRepresentation> point_representation_ptr = boost::make_shared<const MyPointRepresentation> (point_representation);
//   reg.setPointRepresentation (point_representation_ptr);
	
	//TRANSFORMATION ESTIMATION METHOD
// 	TransformationEstimationPointToPlaneLLS< PointXYZNormal,PointXYZNormal > t_est; 
// 	boost::shared_ptr<const TransformationEstimationPointToPlaneLLS< PointXYZNormal,PointXYZNormal> > t_est_ptr = boost::make_shared<const TransformationEstimationPointToPlaneLLS< PointXYZNormal,PointXYZNormal> > (t_est);
	
	typedef typename pcl::registration::TransformationEstimationPointToPlaneLLS< PointXYZNormal,PointXYZNormal> regtype;
	
	boost::shared_ptr< regtype> te (new regtype);
// 	te->setWarpFunction (warp_fcn);
	
	
// 	reg.setTransformationEstimation(te);
	reg.setTransformationEpsilon (transformation_epsilon);
	reg.setMaxCorrespondenceDistance (max_correspondence_distance);
	reg.setMaximumIterations (max_itern);
// 	reg.setRANSACOutlierRejectionThreshold (ransac_threshold);
// 	reg.setEuclideanFitnessEpsilon (euclidean_fitness);
// 	reg.setRANSACIterations(ransac_iterations);
	
 	reg.setInputCloud (cloud_with_normals);
 	reg.setInputTarget (tcloud_with_normals);
	
	
	printf("Aligning\n");
	pcl::PointCloud<PointXYZNormal>::Ptr reg_result (new pcl::PointCloud<PointXYZNormal>);
	reg.align(*reg_result);
	
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity ();
	Ti = reg.getFinalTransformation ();
	
	std::cout << "has converged:" << reg.hasConverged() << " score: " << reg.getFitnessScore() << std::endl;
	std::cout << Ti << std::endl;

	
	printf("Saving results\n");
// 	Cloud::Ptr out_cloud (new Cloud);
 	pcl::io::savePCDFileBinary("Output.pcd", *reg_result);
	
	//we just need xyz data
// 	pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
// 	pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZ>(*out_cloud, xyz_cloud);
// 	pcl::transformPointCloud (*cloud, *out_cloud, Ti);
// 	pcl::io::savePCDFileBinary("Output2.pcd", *out_cloud);
	
	
	
// 	std::string infilename = cloudname;
// 	std::string outfilename = stripExtension(infilename) + "_finally_aligned.pcd";
	
// 	saveKeepingFields(*out_cloud, infilename, outfilename);
	
	return (0);
}
