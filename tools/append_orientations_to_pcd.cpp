#include<pcl/io/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>

#include <spc/common/strings.h>

#include <pcl/features/normal_3d.h>

#include <spc/methods/compute_eigen_indices.h>
#include <spc/common/io_helper.h>


using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;




void printHelp(int argc, char ** argv)
{
//    print_info("USAGE: ll_compute_normals_eigenvalues incloud.pcd outcloud.pcd  [options]\n");
//    print_info("Computes normals and indices of point dispersion as ratio of eigenvalues of local covariance matrix\n");
//    print_info("Options are:\n");
//    print_info("-v verbose output\n");
//    print_info("-t int number of threads to be used\n");
//    print_info("-r radius used for neighbors search\n");
//    print_info("-c concatenate the original fields in the indices files, add also normals and eigenvalues - save everything\n");
//    print_info("-a save as ascii pcd file instead of binary compressed (default)\n");
//    print_info("-h this help\n");

}


Eigen::Vector3f vector_from_string(const string line, const string separator)
{
    vector<string> strs;
    boost::split(strs,line,boost::is_any_of(" "));

    Eigen::Vector3f out;
    for (int i = 0 ; i < 3; ++i)
        out(i) = atof(strs.at(i).c_str());

    return out;
}

int main (int argc, char ** argv)
{
    pcl::console::TicToc tt;
    tt.tic();

    float def_radius = 0.1;
    int def_n_threads = 2;


    //ids of the cloud files
    vector<int> ids = parse_file_extension_argument (argc, argv, string("pcd"));


    BOOST_FOREACH (int i, ids)
    {
        string cloudfname = argv[i];

        print_info("Loading %s\n", cloudfname.c_str());

        string basename = boost::filesystem::basename(cloudfname);
        string orientation_file = basename + ".txt"; //we expect to be txt

        print_info("Looking for orientation file %s \n", orientation_file.c_str());

        ifstream file (orientation_file);
        if (!file.is_open())
        {
            print_error("Cannot locate the orientation file\n");
            return -1;
        }

        string line;
        getline(file, line);

        std::cout << std::endl;

        Eigen::Vector3f position = vector_from_string(line, " ");

        getline(file, line);
        Eigen::Vector3f axis = vector_from_string(line, " ");

        getline(file, line);
        float angle  = atof(line.c_str());

        print_info("Found position: \n");
        std::cout << position << std::endl;

        print_info("Found axis: \n");
        std::cout << axis << std::endl;

        print_info("Found angle: \n");
        std::cout << angle << std::endl;

        std::cout << std::endl;

        file.close();


        Eigen::Vector4f origin;
        for (int i = 0; i < 3; ++i)
            origin(i) = position(i); //simply copying

         origin(3) = 0; //setting to zero the last component




        Eigen::AngleAxisf ang_axis (angle*M_PI/180.0, axis);

        Eigen::Quaternionf orientation(ang_axis);


        print_info("loading the cloud now\n");
        pcl::PCLPointCloud2 in_cloud_sr;
        loadPCDFile(cloudfname, in_cloud_sr);

        print_info("saving the cloud now\n");


        auto out_name = basename + "_ori.pcd";


        savePCDFile (out_name, in_cloud_sr,
                     origin,
                     orientation,
                     true);


    }


//   pcl::PCLPointCloud2 in_cloud_sr;
//    loadPCDFile(in_cloud_name, in_cloud_sr, origin, orientation);

//    cout << "Radius: " << radius << endl;
//    cout << "Starting " << n_threads << " threads" << endl;

//    PointCloud<PointXYZ>::Ptr in_cloud (new PointCloud<PointXYZ>); //we just need for xyz in pcl format
//    fromPCLPointCloud2(in_cloud_sr, *in_cloud);


//    print_info("Number of points in input: "); print_value( "%i" ,in_cloud->size()); print_info("\n");

//    pcl::console::print_highlight("Computing normals and eigenvalues\n");
//    PointCloud<PointNormalEigs> eigenvalues_cloud;
//    spc::computeNormalsAndEigenvalues <pcl::PointXYZ, PointNormalEigs> (in_cloud, radius, n_threads, eigenvalues_cloud);
//    print_info("Finished in "); tt.toc_print(); print_info("\n");


//    pcl::console::print_highlight("Computing indices\n");
//    PointCloud<PointEigIndices> indices_cloud; //cloud for storing indices
//    spc::computeDispersionIndices(eigenvalues_cloud, indices_cloud, n_threads);

//    pcl::console::print_highlight("Saving results\n");

//   pcl::PCLPointCloud2 out_indices_sensor;
//    pcl::toPCLPointCloud2(indices_cloud, out_indices_sensor);

//    if (find_switch(argc, argv, "-c") == -1)
//    {
//        if (find_switch(argc, argv, "-a"))
//            savePCDFile(out_cloud_name, out_indices_sensor);
//        else
//            spc::savePCDBinaryCompressed(out_cloud_name, out_indices_sensor);
//    }
//    else
//    {
//       pcl::PCLPointCloud2 out_all_sensor, eigenvalues_cloud_sm, out_indices_sm;
//        concatenateFields(in_cloud_sr, out_indices_sensor, out_indices_sm);

//        pcl::toPCLPointCloud2(eigenvalues_cloud, eigenvalues_cloud_sm);

//        concatenateFields(eigenvalues_cloud_sm, out_indices_sm, out_all_sensor);
//        if (find_switch(argc, argv, "-a"))
//            savePCDFile(out_cloud_name, out_indices_sensor);
//        else
//            spc::savePCDBinaryCompressed(out_cloud_name, out_all_sensor);

//    }

    return 1;
}
