#include<pcl/io/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <spc/core/spc_eigen.h>

#include <spc/core/spc_strings.h>

#include <pcl/features/normal_3d.h>

#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <spc/io/io_helper.h>


using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;




void printHelp(int argc, char ** argv)
{
    print_info("USAGE: %s incloud1.pcd [incloud2.pcd ... incloudN.pcd]  [options]\n", argv[0]);
    print_info("This tool will search for a incloud1.txt file with orientation information inside\n");
    print_info("The program expect to find a text file (.txt) with the same name of the .pcd cloud\n");
    print_info("With three lines with the following sytax:\n");
    print_info("x y z               <- the position of the sensor\n");
    print_info("ax_x ax_y ax_z      <- the axis orienting the view of the scanner\n");
    print_info("angle [in degrees]  <- an angle of rotation around the orientating axis\n");
    print_info("Options are:\n");
//    print_info("-p position only\n");
    print_info("-h this help\n");
    print_info("Output a new pcd cloud with the _ori suffix for each in cloud (will OVERWRITE existent files)\n");


}


 bool vector_from_string(const string line, const string separator, Eigen::Vector3f & v)
{
    vector<string> strs;
    boost::split(strs,line,boost::is_any_of(" "));

    if (strs.size() < 3)
    {
        print_warn("Cannot translate line to vector\n");
        v = Eigen::Vector3f::Zero();
        return false;

    }


    for (int i = 0 ; i < 3; ++i)
        v(i) = atof(strs.at(i).c_str());

    return true;
}

bool parse_orientation_file(const string filename, Eigen::Vector4f &origin, Eigen::Quaternionf &orientation)
{

    ifstream file (filename);
    if (!file.is_open())
    {
        print_error("Cannot locate the orientation file\n");
        return false;
    }

    string line;
    getline(file, line);

    std::cout << std::endl;

    Eigen::Vector3f position;
    bool found_po = vector_from_string(line, " ", position);

    getline(file, line);
    Eigen::Vector3f axis;
    bool found_ax= vector_from_string(line, " ", axis);

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


//    Eigen::Vector4f origin;
    for (int i = 0; i < 3; ++i)
        origin(i) = position(i); //simply copying

     origin(3) = 0; //setting to zero the last component




    Eigen::AngleAxisf ang_axis (angle*M_PI/180.0, axis);

    orientation = Eigen::Quaternionf (ang_axis);


}

INITIALIZE_EASYLOGGINGPP

int main(int argc, char ** argv)
{
	START_EASYLOGGINGPP(argc, argv);
    pcl::console::TicToc tt;
    tt.tic();

    float def_radius = 0.1;
    int def_n_threads = 2;


    //ids of the cloud files
    vector<int> ids = parse_file_extension_argument (argc, argv, string("pcd"));

    if (ids.size() == 0)
    {
        printHelp(argc, argv);
        return -1;
    }

    bool only_positions = find_switch(argc, argv, "-p");
    if (only_positions)
    {
        print_warn("Going to search and append only positions to the clouds\n");
    }



    for (int i: ids)
    {
        string cloudfname = argv[i];

        print_info("Loading %s\n", cloudfname.c_str());

        string basename = boost::filesystem::basename(cloudfname);
        string orientation_file = basename + ".txt"; //we expect to be txt

        print_info("Looking for orientation file %s \n", orientation_file.c_str());

        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;

        bool found_ori = parse_orientation_file(orientation_file, origin, orientation);

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
