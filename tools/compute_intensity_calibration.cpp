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
#include <spc/calibration/CalibrationDataEstimator.h>
#include <spc/calibration/CalibrationDataFilter.h>

#include <boost/spirit/home/support/detail/hold_any.hpp>
#include <spc/calibration/IntensityCalibrator.h>
#include <spc/calibration/IntensityCalibratrionModels.h>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;




void printHelp(int argc, char ** argv)
{
    print_info("USAGE: %s dataset.txt  [options]\n", argv[0]);
    print_info("Computes calibration for the given dataset computed with compute_calibration_dataset\n");
    print_info("Options are:\n");
    print_info("-mcp use a different multipler for each input cloud. Default = No\n");

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

    setVerbosityLevel(VERBOSITY_LEVEL::L_DEBUG);


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

    bool mult_use_diff = find_switch(argc, argv, "-mcp");




    //ids of the cloud files
    vector<int> ids = parse_file_extension_argument (argc, argv, string("txt"));
    std::string dataset_file = argv[ids.at(0)];

    print_info("Loading the calibration dataset from %s\n", dataset_file.c_str());

    spc::CalibrationDataDB::Ptr db (new spc::CalibrationDataDB);
    db->fromFile(dataset_file);

    print_info("Found %i core points\n", db->size());

    spc::IntensityCalibratrionModelFactorsBased::Ptr model(new spc::JutzyModel(db, !mult_use_diff));

    spc::IntensityCalibrator calibrator;
    calibrator.setCalibrationDataDB(db);
    calibrator.setModel(model);
    calibrator.optimize();

    Eigen::VectorXf out = model->getPredictedIntensities(db);

    ofstream myfile;
      myfile.open ("_predicted.txt");
      myfile << out;
      myfile.close();

       out = model->getCorrectedIntensities(db);

        myfile.open ("_corrected.txt");
        myfile << out;
        myfile.close();

//    std::cout << out << std::endl;





    return 1;
}
