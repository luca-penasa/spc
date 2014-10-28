#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
struct PointStats
	{
		PCL_ADD_POINT4D;
		float intensity;
		float mean;
		float std;
		
		
 		
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
	} EIGEN_ALIGN16;

	POINT_CLOUD_REGISTER_POINT_STRUCT   (PointStats,
					     (float, x, x)
					     (float, y, y)
					     (float, z, z)
					     (float, intensity, intensity)
						(float, mean, mean)
						(float, std, std)
					    
	)

typedef PointStats PointS;
typedef pcl::PointXYZI PointT;

float G(float x, float sigma)
{
	return exp(-(x*x)/(2*sigma*sigma));
}

int main(int argc, char *argv[])
{
	std::string incloudfile = argv[1];
	std::string outcloudfile = argv[2];
	float radius = atof(argv[3]);
	
	//Load cloud
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(incloudfile.c_str(), *cloud);
	
	pcl::PointCloud<PointS>::Ptr outcloud (new pcl::PointCloud<PointS>);
	pcl::copyPointCloud(*cloud, *outcloud);
	
	//TODO Destruct the first point cloud
	
	int pnumber = (int) outcloud->size();
	
	
	//Set up KDTree
	pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
	tree->setInputCloud (cloud);




	
	//Neighbors containers
	std::vector< int > k_indices ;
	std::vector< float > k_distances ;
	
	//Some Variables
	float mean;
	float std;
	int id;
	float intensity;
	
	//Main Loop
	for (int point_id = 0; point_id < pnumber; ++point_id)
	{
                k_indices.clear();
                k_distances.clear();
                mean = 0.0;
                std = 0.0;

		tree->radiusSearch(point_id, radius, k_indices, k_distances);
		int k_number = k_indices.size();
		
		{
			//For each neighbor
			for (int n_id = 0; n_id < k_number; ++n_id)
			{
				id = k_indices.at(n_id);
				intensity = outcloud->points[id].intensity;
				mean += intensity;
			}
			mean = mean / k_number;
			outcloud->points[point_id].mean = mean;
			
			//For each neighbor
			for (int n_id = 0; n_id < k_number; ++n_id)
			{
				id = k_indices.at(n_id);
				intensity = outcloud->points[id].intensity;
                                std += pow(intensity - mean, 2);
			}
			std = sqrt(std / k_number);
			outcloud->points[point_id].std = std;
			
		
		}

	}
	
	//Save filtered output
	pcl::io::savePCDFile(outcloudfile.c_str(), *outcloud);
	return 0;
}
