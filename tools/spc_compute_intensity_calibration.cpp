#include <spc/core/spc_eigen.h>


#include<pcl/io/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <spc/core/spc_eigen.h>
#include <iostream>
#include <spc/core/strings.h>


#include <pcl/features/normal_3d.h>

#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <spc/io/io_helper.h>


//#include <spc/methods/EigenFunctionsParametrizator.h>
#include <spc/methods/IntensityCalibrationDataEstimator.h>
//#include <spc/methods/IntensityCalibrationDataFilter.h>

#include <boost/spirit/home/support/detail/hold_any.hpp>
//#include <spc/experimental/IntensityCalibration.h>
//#include <spc/experimental/ICalModelFactors.h>
#include <spc/io/element_io.h>


//#include <spc/experimental/ICalPHFunction.h>
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

    std::cout << "starting routine" << std::endl;


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
    vector<int> ids = parse_file_extension_argument (argc, argv, string("spc"));
    std::string dataset_file = argv[ids.at(0)];

    print_info("Loading the calibration dataset from %s\n", dataset_file.c_str());

//     (new spc::SamplesDB);

    spc::ISerializable::Ptr el  = spc::io::deserializeFromFile(dataset_file);
    spc::EigenTable::Ptr db = spcDynamicPointerCast<spc::EigenTable> (el);



    db = db->getWithStrippedNANs({"distance", "angle"}); //strip if these fields are nans

    Eigen::VectorXf distances = db->column("distance");
    Eigen::VectorXf angles = db->column("angle");
    Eigen::VectorXf intensity = db->column("intensity");

    Eigen::MatrixXf vars(distances.size(), 2);
    vars.col(0) = distances;
//    vars.col(1) = angles;

    for (int i = 0; i < angles.size(); ++i)
        vars(i, 1) = cos(DEG2RAD (angles(i)));


    std::cout << "stripped stuff" << std::endl;


//    spc::ICalPHFunction::Ptr function(new  spc::ICalPHFunction);

//    spc::EigenFunctionsParametrizator estimator;
//    estimator.setFunction(function);
//    estimator.setY(intensity);
//    estimator.setVariables(vars);

//    std::cout << "before compute" << std::endl;

//    estimator.compute();

//    std::cout << "after compute" << std::endl;
//    std::cout << function->getCoefficients() << std::endl;




//    spc::io::serializeToFile(function,"calibration_model", spc::io::XML);








    return 1;
}
