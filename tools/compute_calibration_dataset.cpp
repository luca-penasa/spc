#include<pcl/io/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>

#include <spc/methods/strings.h>

#include <pcl/features/normal_3d.h>

#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <spc/io/io_helper.h>
#include <spc/methods/IntensityCalibrationDataEstimator.h>
#include <spc/methods/IntensityCalibrationDataFilter.h>

#include <boost/spirit/home/support/detail/hold_any.hpp>

#include <spc/io/element_io.h>
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
    print_info("-nc cloud.pcd is a cloud with normals from witch to get normals for angles computation (optional)\n");
    print_info("-sr float is the search radius to use when normals are estimated directly (optional. defualt is 0.1)\n");
    print_info("-md maximum accettable distance for normal transferring when using -nc options (precompute normals)\n");

    print_info("-g use gaussian filtering for estimating the average intensity\n");
    print_info("-ss the spatial sigma for intensity sampling with gaussian sampling\n");

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

    if (argc == 1)
    {
        printHelp(argc, argv);
        print_error("not enough args\n");
        return 1;
    }

    bool ask_help = find_switch(argc, argv, "-h");
    if (ask_help)
    {
        print_error("Here is your help\n");
        printHelp(argc, argv);
        return 1;
    }




    //ids of the cloud files
    vector<int> ids = parse_file_extension_argument (argc, argv, string("pcd"));
    std::string core_point_file;
    parse_argument (argc, argv, "-cp", core_point_file);

    std::string normal_cloud_file;
    parse_argument (argc, argv, "-nc", normal_cloud_file);

    float search_radius = 0.1;
    parse_argument (argc, argv, "-sr", search_radius);

    float max_distance = 0.1;
    parse_argument (argc, argv, "-md", max_distance);

    bool use_gaussian = find_switch(argc, argv, "-g");

    float gaussian_kernel_size(0.1);
    parse_argument(argc, argv, "-ss", gaussian_kernel_size);

    std::vector<std::string> cloud_names;

    BOOST_FOREACH (int i, ids)
    {
        string cloudfname = argv[i];
        if (cloudfname != core_point_file & cloudfname != normal_cloud_file)
            cloud_names.push_back(cloudfname);
    }

    spc::CalibrationDataEstimator calibrator;
    calibrator.setInputClouds(cloud_names);

    if (!normal_cloud_file.empty())
    {
        calibrator.setInputNormalsCloudName(normal_cloud_file);
        calibrator.setNormalEstimationMethod(spc::CalibrationDataEstimator::PRECOMPUTED_NORMALS);
        calibrator.setMaximumDistanceForGettingNormal(max_distance);
    }

    calibrator.setInputSamples(core_point_file);

    calibrator.setSearchRadius(search_radius);

    if (use_gaussian)
    {
        calibrator.setIntensityEstimationMethod(spc::CalibrationDataEstimator::GAUSSIAN_ESTIMATION);
        calibrator.setIntensityGaussianSpatialSigma(gaussian_kernel_size);
    }

    calibrator.compute();



    spc::EigenTable::Ptr db = calibrator.getCalibrationDB();
//    db = db.getValidDataOnly(); //filter out nans

    //    db.printOutStuff();

    //we dont filter out northing if we are using precomputed normals

    if (normal_cloud_file.empty())
    {
        pcl::console::print_info("Finished computing core points data\n");

        pcl::console::print_info("Starting to filter core points BUT NOT IMPLEMENTED\n");

//        //now filter out normal
//        spc::CalibrationDataFilter f;
//        pcl::console::print_info("chosing an unique normal for each core point\n");

//        f.setInputCalibrationSamplesDB(db);
//        f.fixUniqueNormals();
//        pcl::console::print_info("Done\n");
//        pcl::console::print_info("Now we recompute all the scattering angles\n");
//        f.recomputeScatteringAngles();
//        pcl::console::print_info("Done\n");

    }




    spc::io::serializeToFile(db, "./out", spc::io::XML);


    return 1;
}
