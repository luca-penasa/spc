#include<pcl/io/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <spc/methods/spc_eigen.h>

#include <spc/methods/strings.h>

#include <pcl/features/normal_3d.h>

#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <spc/io/io_helper.h>


using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;




void printHelp(int argc, char ** argv)
{
    print_info("USAGE: ll_compute_normals_eigenvalues incloud.pcd outcloud.pcd  [options]\n");
    print_info("Computes normals and indices of point dispersion as ratio of eigenvalues of local covariance matrix\n");
    print_info("Options are:\n");
    print_info("-v verbose output\n");
    print_info("-t int number of threads to be used\n");
    print_info("-r radius used for neighbors search\n");
    print_info("-c concatenate the original fields in the indices files, add also normals and eigenvalues - save everything\n");
    print_info("-a save as ascii pcd file instead of binary compressed (default)\n");
    print_info("-h this help\n");

}

int main (int argc, char ** argv)
{
    pcl::console::TicToc tt;
    tt.tic();

    float def_radius = 0.1;
    int def_n_threads = 2;


    //ids of the cloud files
    vector<int> ids = parse_file_extension_argument (argc, argv, string("pcd"));


    if (find_switch(argc, argv, "-h"))
    {
        printHelp(argc, argv);
        return 1;
    }

    if (ids.size() != 2)
    {
        print_error("Error, provide input and output pcd files\n");
        printHelp(argc, argv);
        return 0;
    }

    string in_cloud_name = argv[ids[0]]; //first passed cloud is input
    string out_cloud_name =  argv[ids[1]]; //the second one is output

    cout << "Working on file: " << in_cloud_name << endl;
    //the radius
    float radius;
    if (parse_argument(argc, argv, "-r", radius) == -1)
        radius = def_radius;

    int n_threads;
    if (parse_argument(argc, argv, "-t", n_threads) == -1)
        n_threads = def_n_threads;

    bool verbose;
    if (find_switch(argc, argv, "-v"))
        verbose = true;
    else
        verbose = false;




    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;

   pcl::PCLPointCloud2 in_cloud_sr;
    loadPCDFile(in_cloud_name, in_cloud_sr, origin, orientation);

    cout << "Radius: " << radius << endl;
    cout << "Starting " << n_threads << " threads" << endl;

    PointCloud<PointXYZ>::Ptr in_cloud (new PointCloud<PointXYZ>); //we just need for xyz in pcl format
    fromPCLPointCloud2(in_cloud_sr, *in_cloud);


    print_info("Number of points in input: "); print_value( "%i" ,in_cloud->size()); print_info("\n");

    pcl::console::print_highlight("Computing normals and eigenvalues\n");
    PointCloud<PointNormalEigs> eigenvalues_cloud;
    spc::computeNormalsAndEigenvalues <pcl::PointXYZ, PointNormalEigs> (in_cloud, radius, n_threads, eigenvalues_cloud);
    print_info("Finished in "); tt.toc_print(); print_info("\n");


    pcl::console::print_highlight("Computing indices\n");
    PointCloud<PointEigIndices> indices_cloud; //cloud for storing indices
    spc::computeDispersionIndices(eigenvalues_cloud, indices_cloud, n_threads);

    pcl::console::print_highlight("Saving results\n");

   pcl::PCLPointCloud2 out_indices_sensor;
    pcl::toPCLPointCloud2(indices_cloud, out_indices_sensor);

    if (find_switch(argc, argv, "-c") == -1)
    {
        if (find_switch(argc, argv, "-a"))
            savePCDFile(out_cloud_name, out_indices_sensor);
        else
            spc::io::savePCDBinaryCompressed(out_cloud_name, out_indices_sensor);
    }
    else
    {
       pcl::PCLPointCloud2 out_all_sensor, eigenvalues_cloud_sm, out_indices_sm;
        concatenateFields(in_cloud_sr, out_indices_sensor, out_indices_sm);

        pcl::toPCLPointCloud2(eigenvalues_cloud, eigenvalues_cloud_sm);

        concatenateFields(eigenvalues_cloud_sm, out_indices_sm, out_all_sensor);
        if (find_switch(argc, argv, "-a"))
            savePCDFile(out_cloud_name, out_indices_sensor);
        else
            spc::io::savePCDBinaryCompressed(out_cloud_name, out_all_sensor);

    }

    return 1;
}
