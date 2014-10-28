#include <lidarlib.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <flann/flann.hpp>

#include "pcl/kdtree/kdtree_flann.h"
int
main (int argc, char *argv[])
{
	std::string infilename = argv[1]; 
	
	//load the cloud 
// 	sensor_msgs::PointCloud2::Ptr sensor_d (new sensor_msgs::PointCloud2);
	pcl::PointCloud<PointD>::Ptr distances (new pcl::PointCloud<PointD>);
	pcl::io::loadPCDFile(infilename.c_str(), *distances);
	
	pcl::KdTreeFLANN<PointD> kdtree;

	kdtree.setInputCloud (distances);

	PointD searchPoint;

	searchPoint.distance = 1;

	//now try a search
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 10;

	std::cout << "Neighbors within radius search at (" << searchPoint.distance 
		<< ") with radius=" << radius << std::endl;


	kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	
	pcl::console::print_info("We got %i neighbors\n", pointIdxRadiusSearch.size());
	
	for (int i =0; i< 10; ++i)
	{
		std::cout << pointIdxRadiusSearch[i] << "\t " << sqrt(pointRadiusSquaredDistance[i]) << std::endl;
	}
	//TRY WITH JUST FLANN IF YOU CAN!
	//write distances to a vector 
	long int n_points = distances->points.size();
	long int dim = 1; //dimensions of dataset
	float v_dist[n_points];
	
	for (int i=0; i < n_points; ++i)
	{
		v_dist[i] =distances->points[i].distance;
	}
	//initialize the flann index
	float epsilon = 0.0; //error in retrieving points
	typedef flann::L2_Simple<float> dist;
	typedef flann::Index<dist> FLANNIndex;

	FLANNIndex* flann_index = new FLANNIndex (flann::Matrix<float> (v_dist, n_points, dim), flann::KDTreeSingleIndexParams (15));
	flann_index->buildIndex();
	
	float pos = searchPoint.distance;
	//radius was yet declared
	
	
 	radius *= radius; //flann need squared radius
	
	static flann::Matrix<int> indices_empty;
	static flann::Matrix<float> dists_empty;
	
	int neighbors_in_radius = 0;
	bool sorted = true;
	flann::SearchParams param_radius = flann::SearchParams (-1 ,epsilon, sorted);
	std::vector<float> tmp (dim);
	tmp[0] = pos;
	
	//first search for allocate dimensions
	neighbors_in_radius = flann_index->radiusSearch 
	(flann::Matrix<float>(&tmp[0], 1, dim),
		indices_empty, dists_empty, radius, 
		param_radius);
	
	std::vector<int> k_indices;
	std::vector<float> k_squared_distances;
	
	k_indices.resize (neighbors_in_radius);
	k_squared_distances.resize (neighbors_in_radius);
	
	std::cout << "with this method:" << neighbors_in_radius << std::endl;
	
	//now perform actual search
	//declare flann::Matrix of right size
	
// 	flann::Matrix<int> k_indices_mat (new int[k_indices.size()*1], 1, k_indices.size ());
// 	flann::Matrix<float> k_distances_mat (new float[k_indices.size()*1], 1, k_indices.size ());
	flann::Matrix<int> k_indices_mat (&k_indices[0], 1, k_indices.size ());
	flann::Matrix<float> k_distances_mat (&k_squared_distances[0], 1, k_squared_distances.size ());
	//and do search
	int new_number = flann_index->radiusSearch (flann::Matrix<float>(&tmp[0], 1, dim),
                                k_indices_mat, k_distances_mat, radius, 
                                param_radius);
  
	
		for (int i =0; i< 10; ++i)
	{
		std::cout << k_indices[i] << "\t " << sqrt(k_squared_distances[i]) << std::endl;
	}
	
	
	std::cout << new_number << std::endl;
	//NOW RECREATE USING FLANN
		
	return 1;
}