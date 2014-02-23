#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <pcl/common/common.h>

struct PointXYZId
    {
        PCL_ADD_POINT4D;
        unsigned int id;




        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    POINT_CLOUD_REGISTER_POINT_STRUCT   (PointXYZId,
                         (float, x, x)
                         (float, y, y)
                         (float, z, z)
                         (int, id, id)


    )



bool verbose = false;

void printHelp()
{
    pcl::console::print_info("USAGE: extract_pois points.pcd cloud.pcd out_filename.pcd\n");
    pcl::console::print_info("Take as input a set of points in pcd format and a cloud.\n");
    pcl::console::print_info("Search into the cloud for the nearest neighbor of each point\n");
    pcl::console::print_info("Output is a point cloud with XYZ and Id\n");
    pcl::console::print_info("Where XYZ correspond to the points and the Id is the nearest point in cloud\n");
}

int main (int argc, char *argv[])
{

    std::vector<int> ids = pcl::console::parse_file_extension_argument (argc, argv, std::string("pcd"));

//    for (auto i : ids)
//        std::cout << i << std::endl;

//    std::cout << ids.size() << std::endl;


    bool help_request = pcl::console::find_switch (argc, argv, "-h");




    if (ids.size()!= 3)
    {
        std::cout << "SOME ERROR OCCOURED. PLEASE CHECK SYNTAX, use -h for help" << std::endl;
        printHelp();
        return -1;
    }
    else if (help_request)
    {
        printHelp();
        return -1;
    }




    verbose = pcl::console::find_switch (argc, argv, "-v");

    std::string points_name = argv[ids[0]];
    std::string cloud_name = argv[ids[1]];

    std::string out_name = argv[ids[2]];

    pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(points_name, *points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(cloud_name, *cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> flann;
    flann.setInputCloud(cloud);

    std::vector<int> indices;
    std::vector<float> distances;

    pcl::PointCloud<PointXYZId>::Ptr cloud_indices (new pcl::PointCloud<PointXYZId>);
    copyPointCloud(*points, *cloud_indices);


    int counter = 0;
    for (int i = 0; i < points->size(); ++i)
    {
        pcl::PointXYZ point = points->at(i);
        flann.nearestKSearch(point, 1, indices, distances);


        if (verbose)
        {
            std::cout << "Looking for point " << point << std::endl;
            std::cout <<  " --> Found with id: " << indices[0] << std::endl;
            std::cout <<  " --> With distance: " << std::sqrt(distances[0]) << std::endl;
        }



        (*cloud_indices)[counter].id = indices[0];



        counter++;
    }

    pcl::io::savePCDFileASCII(out_name, *cloud_indices);

    return 1;
}
