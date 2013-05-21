#include <spc/common/std_helpers.hpp>
#include <vector>

#include <pcl/console/print.h> //for printing
#include <pcl/console/parse.h> //for parsing
#include <pcl/io/pcd_io.h>

#include <spc/common/strings.h>

#include <spc/common/geometry.h>

std::string default_merge = "yes";
int default_binary = 1;

void 
printHelp(int argc, char *argv[])
{
	pcl::console::print_error("Syntax is ...\n");
	
	pcl::console::print_info("Options are:\n");
	pcl::console::print_info("            -m yes or no. Say if the output will be merged in just one unique cloud.    Default %s\n", default_merge.c_str());
	pcl::console::print_info("            -b 0/1 if results must be saved in binary mode.                             Default %i\n", default_binary);
	
}

int main(int argc, char* argv[])
{
	
	//parse input pcd files
	std::vector<int> file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	
	if (file_indices.size() == 0)
	{
		pcl::console::print_error("No pcd files given.\n");
		printHelp(argc, argv);
	}
	
	//check if merge the distance cloud with the input one
	std::string merge;
	int binary = default_binary;
	pcl::console::parse_argument (argc, argv, "-m", merge);
	pcl::console::parse_argument (argc, argv, "-b", binary);
	
	
	bool is_binary;
	if (binary == 0)
	{
		is_binary = false;
	}
	else if (binary == 1)
	{
		is_binary = true;
	}
	else
	{
		pcl::console::print_error("-b argument must be 0 or 1.\n");
		return (-1);
		
	}
	
	//put filenames in a vector of strings
	std::vector<std::string> filenames;
	for (int i = 0; i < file_indices.size(); ++i)
	{
		filenames.push_back(argv[file_indices[i]]);	
		pcl::console::print_info("-> Found file: %s\n", filenames[i].c_str());
	}
	
	//create a cloud for storing data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2::Ptr in_sensor_cloud (new sensor_msgs::PointCloud2);
	sensor_msgs::PointCloud2::Ptr out_sensor_cloud (new sensor_msgs::PointCloud2);
	pcl::PointCloud<PointD>::Ptr cloud_distances (new pcl::PointCloud<PointD>);
	sensor_msgs::PointCloud2::Ptr cloud_distances_sensor (new sensor_msgs::PointCloud2);
	//now for each cloud:
	for (int i = 0; i < filenames.size(); ++i)
	{	
		//load the pcd file
		pcl::io::loadPCDFile(filenames[i].c_str(), *in_sensor_cloud);
		
		//check if the loaded cloud have an x field
		if (pcl::getFieldIndex(*in_sensor_cloud, "x") == -1)
		{
			pcl::console::print_error ("Input cloud %s does not have gemetrical fields. Skipping.\n", filenames[i].c_str());
			continue;
		}
		
		pcl::fromROSMsg(*in_sensor_cloud, *cloud);
		//compute distances
        spc::computeDistanceFromOrigin(*cloud, *cloud_distances);
		pcl::toROSMsg(*cloud_distances, *cloud_distances_sensor);
		
		merge = default_merge;
		
		if (merge == "yes")
		{
            std::string base_name = spc::getBaseFileName(filenames[i].c_str());
			std::string out_name = base_name + "_WithDistances.pcd"; 
			pcl::concatenateFields(*cloud_distances_sensor, *in_sensor_cloud, *out_sensor_cloud);
			pcl::io::savePCDFile (out_name, *out_sensor_cloud, Eigen::Vector4f::Zero (),Eigen::Quaternionf::Identity (), is_binary);	
			pcl::console::print_info("Saved as %s\n", out_name.c_str() );
			
		}
		else if (merge == "no")
		{
            std::string base_name = spc::getBaseFileName(filenames[i].c_str());
			std::string out_name = base_name + "_OnlyDistances.pcd";
			pcl::io::savePCDFile (out_name, *cloud_distances_sensor, Eigen::Vector4f::Zero (),Eigen::Quaternionf::Identity (), is_binary);	
			pcl::console::print_info("Saved as %s\n", out_name.c_str() );
		}
		else
		{
			pcl::console::print_error("Merging switch only accept \"yes\" or \"no\"\n");
			return -1;
		}
		

		
	}
	
	
	return 1;
}
