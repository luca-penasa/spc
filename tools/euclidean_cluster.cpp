#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <boost/make_shared.hpp>


#include <pcl/console/print.h> //for printing
#include <pcl/console/parse.h> //for parsing


#include <spc/common/strings.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
// typedef pcl::KdTreeFLANN<PointT> KdTreeT;
typedef pcl::PointIndices PointIndicesT;

using namespace pcl::console;

double default_tolerance = 0.1;
bool default_save_clustered = false;

struct PointXYZId
	{
		PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
		int id;
		float intensity;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
	}  EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

	POINT_CLOUD_REGISTER_POINT_STRUCT       (PointXYZId,           // here we assume a XYZ + "test" (as fields)
						(float, x, x)
						(float, y, y)
						(float, z, z)
						(int, id, id)
						(float, intensity, intensity)
								)

int 
computeEuclideanClustering(const PointCloudT::Ptr incloud, 
													 const float tolerance, 
													 std::vector<PointIndicesT> &indices
)
{
	//Start
	pcl::EuclideanClusterExtraction<PointT> filter;
	filter.setInputCloud(incloud);
	filter.setClusterTolerance(tolerance);
	filter.extract (indices);
	return 1;
}
int
extractIndices(const PointCloudT::Ptr incloud, 
							 const PointIndicesT indices,
							 PointCloudT::Ptr outcloud
							)
{
	
// 	boost::shared_ptr<std::string> x = boost::make_shared<std::string>("hello, world!");
	//make a shared pointer for the indices
	boost::shared_ptr <PointIndicesT> indices_ptr = boost::make_shared<PointIndicesT>(indices);
	pcl::ExtractIndices<PointT> extractor;
	extractor.setInputCloud(incloud);
	extractor.setIndices(indices_ptr);
	extractor.filter(*outcloud);
// 	print_info("out cloud have %i points", outcloud->points.size());
}


void printHelp(int argc, char* argv[])
{
	print_info("Syntax is %s cloud1 [cloud2 cloud3 ...] [opts]\n", argv[0]);
	print_info("Options are: \n");
	print_info("     -t double   set threshold distance (tolerance), default %2.2f \n", default_tolerance);
	print_info("     -sc         save the clustered version of the cloud, default %i \n", default_save_clustered);
	print_info("Output is:\n");
	print_info("cloud1_biggest_cluster.pcd      the biggest cluster of the cloud\n");
	print_info("cloud1_clustered.pcd            the original cloud clustered (for viewing result), only with -c \n");
	print_info("                                colors are randomly chosed for being meaningful\n");
	
	
}

int 
main(int argc, char *argv[])
{
	//input files
	std::vector<int> file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	
	//do a check
	if (file_indices.size() < 1)
	{
		print_error("You must give a pcd file as input!\n");
		printHelp(argc, argv);
		return -1;
	}
	
	//get as a vector of strings
	std::vector<std::string> file_names;
	for (int i = 0; i < file_indices.size(); ++i)
	{
		file_names.push_back(argv[file_indices[i]]);
	}
	
	double tolerance = default_tolerance;
	bool is_tolerance_given = parse_argument (argc, argv, "-t", tolerance);
	if (!is_tolerance_given)
	{
		print_warn("Tolerance not given. Default used %2.2f.\n", tolerance);
		print_warn("You can give one using -t double flag.\n");
	}
	
	//Vector for results
	std::vector< PointIndicesT > idx;

	//do last part of command arg parsing
	bool save_clustered = default_save_clustered;
	bool is_save_clustered_given = find_switch(argc, argv, "-sc");
	if (!is_save_clustered_given)
	{
		print_warn("You can ask the program to save the whole clustering output in a file using -sc flag.\n");
		print_warn("In that case a random integer is assigned to each cluster so that colors are meaningful.\n");
	}
	
	
	//main for
	for (int i = 0; i < file_names.size(); ++i)
	{
		
		
		//get filename
		std::string filename = file_names[i];
		
		//Load cloud
		PointCloudT::Ptr cloud (new PointCloudT);
		pcl::io::loadPCDFile(filename.c_str(), *cloud);
		
		//print info
		print_info("\nProcessing %s: %i points\n", filename.c_str(), cloud->points.size());
		
		//create two output filenames
		//one for biggest cluster
		//one for the clustered version of the cloud
        std::string stripped_name = spc::stripExtension(filename);
		std::string biggest_outcloud_name = stripped_name + "_biggest_cluster.pcd";
		std::string clustered_outcloud_name = stripped_name + "_clustered.pcd";
		
		
		
		//compute 
		computeEuclideanClustering(cloud, tolerance, idx);
		
		
		
		
		
		printf("Showing first 10 clusters and number of points:\n");
		for (int i = 0; i < 10; i++ )
		{
			PointIndicesT this_idx;
			this_idx = idx.at(i);
			size_t this_idx_size = this_idx.indices.size();
			print_info("Cluster number %i: %i points.\n", i, this_idx_size); 
		}
		
		print_info("Extracting the biggest cluster\n");
		PointIndicesT this_idx;
		this_idx = idx.at(0); //is the biggest!
		
		size_t this_idx_size = this_idx.indices.size();
		
		
		//create the out cloud
		pcl::PointCloud<PointT>::Ptr outcloud (new pcl::PointCloud<PointT>);
		extractIndices(cloud, this_idx, outcloud);
		
		print_info("Saving %s\n", biggest_outcloud_name.c_str());
		pcl::io::savePCDFileBinary(biggest_outcloud_name.c_str(), *outcloud);
		
		//if is requested to save the clusterin algorithm output
		if (save_clustered)
		{
			pcl::PointCloud<PointXYZId>::Ptr outcloud2 (new pcl::PointCloud<PointXYZId>);
			pcl::copyPointCloud(*cloud, *outcloud2);
			
			srand((unsigned)time(0));
			int random_integer; 
			for (int i = 0; i < idx.size(); ++i)
			{
				random_integer = (rand()%200)+1;
				PointIndicesT this_idx = idx.at(i);
				for (int j =0; j < this_idx.indices.size(); ++j)
				{
					int this_index = this_idx.indices[j];
					outcloud2->points[this_index].id = random_integer;
				}
			}
			print_info("Saving %s\n", clustered_outcloud_name.c_str());
			pcl::io::savePCDFileBinary(clustered_outcloud_name.c_str(), *outcloud2);
		}
		
	
	
	

	
	
	/*
 	//printf("\n Nr of points in region with ID %i: %i\n", j, this_idx_size);
 		for (int i = 0 ; i < this_idx_size ; i++)
 		{
 			outcloud2->points[this_idx.indices.at(i)].id = j ;
 		}
		*/
	
	
	}
	return 1;
}
	
