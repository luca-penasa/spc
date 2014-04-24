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
#include <spc/devices/IntensityAutoCalibrator.h>

#include <boost/spirit/home/support/detail/hold_any.hpp>


using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;




void printHelp(int argc, char ** argv)
{
    print_info("USAGE: %s incloud1.pcd incloud2.pcd [... .pcd] -cp core_points.pcd  [options]\n", argv[0]);
    print_info("Computes a suitable dataset for clibrating lidar devices\n");
    print_info("Options are:\n");
    print_info("-cp cloud.pcd are the core points (mandatory)\n");
    print_info("-sr float is the search radius to use (optional. defualt is 0.1)\n");
//    print_info("-r radius used for neighbors search\n");
//    print_info("-c concatenate the original fields in the indices files, add also normals and eigenvalues - save everything\n");
//    print_info("-a save as ascii pcd file instead of binary compressed (default)\n");
    print_info("-h this help\n");

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

    if (argc ==     1)
        printHelp(argc, argv);

    bool ask_help = find_argument(argc, argv, "-h");
    if (ask_help)
        printHelp(argc, argv);



    //ids of the cloud files
    vector<int> ids = parse_file_extension_argument (argc, argv, string("pcd"));
    std::string core_point_file;
    parse_argument (argc, argv, "-cp", core_point_file);

    float search_radius = 0.1;
    parse_argument (argc, argv, "-sr", search_radius);

    std::vector<std::string> cloud_names;

    BOOST_FOREACH (int i, ids)
    {
        string cloudfname = argv[i];
        if (cloudfname != core_point_file)
            cloud_names.push_back(cloudfname);
    }

    spc::IntensityAutoCalibrator calibrator;
    calibrator.setInputClouds(cloud_names);
    calibrator.setInputCorePoints(core_point_file);

    calibrator.setSearchRadius(search_radius);

    calibrator.compute();

    spc::CalibrationDataDB db = calibrator.getCalibrationDB();

    //now filter out normal
    spc::CorePointsFilter f;
    f.setInputCalibrationDataDB(db);
    f.fixUniqueNormals();
    f.recomputeScatteringAngles();


    db.writeToAsciiFile("./out.txt");


       return 1;
}
