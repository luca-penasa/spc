#include<pcl/io/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <iostream>
#include <spc/methods/strings.h>

#include <pcl/features/normal_3d.h>

#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <spc/io/io_helper.h>
#include <spc/methods/IntensityCalibrationDataEstimator.h>
#include <spc/methods/IntensityCalibrationDataFilter.h>

#include <boost/spirit/home/support/detail/hold_any.hpp>
#include <spc/methods/IntensityCalibration.h>
#include <spc/elements/ICalModelFactors.h>
#include <spc/io/element_io.h>
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
    vector<int> ids = parse_file_extension_argument (argc, argv, string("xml"));
    std::string dataset_file = argv[ids.at(0)];

    print_info("Loading the calibration dataset from %s\n", dataset_file.c_str());

//     (new spc::SamplesDB);

    spc::ISerializable::Ptr el  = spc::io::deserializeFromFile(dataset_file);
    spc::SamplesDB::Ptr db = spcDynamicPointerCast<spc::SamplesDB> (el);

    spc::SamplesDB cleaned =  db->getNotNanEntries<float>("distance").getNotNanEntries<float>("angle").getNotNanEntries<float>("intensity");

    db =  spcMakeSharedPtrMacro<spc::SamplesDB>(cleaned); //prev db will be delated as no more ref exists.

    if (!db)
        return -1;

    // do some testing

    int id = 450;
    float val = db->at(id)->variantPropertyValue<float>("distance") ;
    float val2 = db->at(id)->variantPropertyValue<float>("intensity") ;
    float val3 = db->at(id)->variantPropertyValue<float>("angle") ;
    int val4 = db->at(id)->variantPropertyValue<int>("cloud_id") ;

   pcl::console::print_info("value: %f\n", val);
   pcl::console::print_info("value: %f\n", val2);
   pcl::console::print_info("value: %f\n", val3);
   pcl::console::print_info("value: %i\n", val4);





//    db->fromFile(dataset_file);

    print_info("Found %i core points\n", db->size());

    spc::IntensityCalibratrionModelFactorsBased::Ptr model(new spc::JutzyModel(db, !mult_use_diff));

    spc::IntensityCalibrator calibrator;
    calibrator.setCalibrationSamplesDB(db);
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
