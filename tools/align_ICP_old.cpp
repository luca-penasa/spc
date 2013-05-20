#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <boost/make_shared.hpp>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
//#include <pcl/registration/gicp.h>

//Some typedefs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> CloudWithNormals;
typedef pcl::PointXYZINormal PointFull;
typedef pcl::PointCloud<PointFull> CloudFull;


int k_nr_points = 20;
double radius = 0.02;
int max_itern = 200;
double max_correspondence_distance = 0.1;
double voxsize = 0.02;
double RANSACOutlierRejectionThreshold = 0.1;
double TransformationEpsilon = 1e-5;
double euclidean_fitness = 1e-5;
class MyPointRepresentation : public pcl::PointRepresentation <PointFull>
{
  using pcl::PointRepresentation<PointFull>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 3;
    //nr_dimensions_ = 3;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointFull &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
//     out[3] = p.curvature;
    //out[4] = p.intensity;
  }
};

int main(int argc, char *argv[])
{
	
// 	pcl::console::setVerbosityLevel(pcl::console::L_DEBUG); 
	pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE); 
	
	
	printf("Load Clouds\n");
	//Creating and loading clouds
	Cloud::Ptr cloud (new Cloud);
	Cloud::Ptr tcloud (new Cloud);
	
	Cloud::Ptr cloud_down (new Cloud);
	Cloud::Ptr tcloud_down (new Cloud);
	std::string tcloudname = argv[1];
	std::string cloudname = argv[2];
	pcl::io::loadPCDFile(tcloudname.c_str(), *tcloud);
	pcl::io::loadPCDFile(cloudname.c_str(), *cloud);
	
	printf("Downsampling clouds\n");
	pcl::VoxelGrid<pcl::PointXYZI> vox_grid;
	vox_grid.setInputCloud (cloud);
	vox_grid.setLeafSize (voxsize, voxsize, voxsize);
	vox_grid.setDownsampleAllData(1);
	vox_grid.filter(*cloud_down);
	
	vox_grid.setInputCloud(tcloud);
	vox_grid.filter(*tcloud_down);
	
	printf("Computing normals\n");
	//Compute normals and curvature
	pcl::KdTreeFLANN<PointT>::Ptr kdtree (new pcl::KdTreeFLANN<PointT>);
        pcl::NormalEstimationOMP<PointT, PointNormalT> normal_estimator;
	CloudWithNormals::Ptr cloud_with_normals (new CloudWithNormals);
	CloudWithNormals::Ptr tcloud_with_normals (new CloudWithNormals);
	
        normal_estimator.setNumberOfThreads(2);
//   normal_estimator.setSearchMethod (*kdtree);
	normal_estimator.setKSearch (k_nr_points);

	normal_estimator.setInputCloud (cloud_down);
	normal_estimator.compute (*cloud_with_normals);
	
	CloudFull::Ptr cloudn (new CloudFull);
	pcl::concatenateFields (*cloud_with_normals,*cloud_down,  *cloudn);
		
	normal_estimator.setInputCloud (tcloud_down);
	normal_estimator.compute (*tcloud_with_normals);
	
	CloudFull::Ptr tcloudn (new CloudFull);
	pcl::concatenateFields (*tcloud_with_normals,*tcloud_down,  *tcloudn);
	
	printf("Normals Computed!\n");
	
 	MyPointRepresentation point_representation;
 	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
 	//float alpha[5] = {1.0, 1.0, 1.0, 1.0, 1.0};
// 	float alpha[4] = {1, 1, 1, 1};
	float alpha[3] = {1, 1, 1};
 	point_representation.setRescaleValues (alpha); 
    //pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal > reg;
	//pcl::IterativeClosestPointNonLinear< pcl::PointXYZINormal, pcl::PointXYZINormal > reg;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> reg;
// 	reg.setTransformationEpsilon (TransformationEpsilon);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (max_correspondence_distance);
	// Set the point representation
	boost::shared_ptr<const MyPointRepresentation> point_representation_ptr = boost::make_shared<const MyPointRepresentation> (point_representation);
 	reg.setPointRepresentation (point_representation_ptr);
 
 	reg.setInputCloud (cloudn);
 	reg.setInputTarget (tcloudn);
	reg.setMaximumIterations (max_itern);
	reg.setRANSACOutlierRejectionThreshold (RANSACOutlierRejectionThreshold );
	
 	reg.setEuclideanFitnessEpsilon (euclidean_fitness);
	
	
	CloudFull::Ptr reg_result (new CloudFull);
	
	printf("Aligning\n");
	reg.align(*reg_result);
	
	std::cout << "has converged:" << reg.hasConverged() << " score: " << reg.getFitnessScore() << std::endl;
	std::cout << reg.getFinalTransformation() << std::endl;
	
	
	printf("Done\n");
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity ();
	Ti = reg.getFinalTransformation ();
	
	printf("Saving results\n");
	Cloud::Ptr out_cloud (new Cloud);
	pcl::transformPointCloud (*cloud, *out_cloud, reg.getFinalTransformation());
	pcl::io::savePCDFileBinary("Output.pcd", *out_cloud);
	
	return (0);
}
