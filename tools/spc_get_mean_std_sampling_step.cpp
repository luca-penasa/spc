#include <spc/methods/std_helpers.hpp>
#include <vector>

#include <pcl/console/print.h> //for printing
#include <pcl/console/parse.h> //for parsing
#include <pcl/io/pcd_io.h>

#include <spc/methods/strings.h>

#include <spc/methods/geometry.h>

#include <spc/methods/PointCloudAverageSamplingStepEstimator.h>

std::string default_merge = "yes";
int default_binary = 1;

void 
printHelp(int argc, char *argv[])
{


    std::string tool_name = argv[0];
    pcl::console::print_info("Compute some stats (mean and std) on point-to-nearest-neighbor distances\n", tool_name.c_str());
    pcl::console::print_info("Syntax: %n incloud.pcd\n", tool_name.c_str());

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
    //	std::string merge;
    //	int binary = default_binary;
    //	pcl::console::parse_argument (argc, argv, "-m", merge);
    //	pcl::console::parse_argument (argc, argv, "-b", binary);


    //put filenames in a vector of strings
    std::vector<std::string> filenames;
    for (int i = 0; i < file_indices.size(); ++i)
    {
        filenames.push_back(argv[file_indices[i]]);
        pcl::console::print_info("-> Found file: %s\n", filenames[i].c_str());
    }

    //create a cloud for storing data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //now for each cloud:
    for (int i = 0; i < filenames.size(); ++i)
    {
        //load the pcd file
        pcl::io::loadPCDFile(filenames[i].c_str(), *cloud);


        spc::EstimateAverageSamplingStep<pcl::PointXYZ> estimator;
        estimator.setInputCloud(cloud);

        estimator.applyFilter();

        pcl::console::print_highlight("Mean:    %f", estimator.getMean());
        pcl::console::print_highlight("Std:     %f", estimator.getStd());

    }


    return 1;
}
