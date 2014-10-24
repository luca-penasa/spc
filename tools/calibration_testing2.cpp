

//#include "ceres/ceres.h"
//#include "glog/logging.h"

//#include <spc/io/element_io.h>
//#include <spc/elements/EigenTable.h>

//#include <spc/io/AsciiEigenTableWriter.h>

//#include <spc/calibration/CalibrationFactors.h>

#include <spc/methods/strings.h>


#include <spc/calibration/CalibrationManager.h>

#include <spc/calibration/LinearCalibrator.h>

#include <spc/methods/RBFModelEstimator.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace spc;
using Eigen::Matrix;


DEFINE_bool(linear, true, "linear way");


DEFINE_string(out, "model.xml", "Out Filename.");
DEFINE_string(distance_field_name, "distance", "The field name of the distance field");
DEFINE_string(intensity_field_name, "intensity", "The field name of the intensity field");
DEFINE_string(angle_field_name, "angle", "The field name of the angle field");
DEFINE_string(intensity_std_field_name, "intensity_std", "The field name of standard deviation of intensity");

DEFINE_bool(only_distance, false, "Calibrate both distance and angle effects");
DEFINE_bool(weights, false, "Use weights for solving the problem");

DEFINE_double(sigma, 0, "Sigma value for the kernel of RBF. If 0 it will be automatically chosen");
DEFINE_double(lambda, 0.01, "Lambda value for regularization (smoothness) of RBF");


DEFINE_uint64(poly_order, 1 , "polynomial order for the polynomial part of RBF");
DEFINE_uint64(n_distance_nodes, 6 , "Number of nodes in the distance space \n The total number of nodes"
                                    "will be n_distance_nodes * n_angle_nodes. So be careful.");
DEFINE_uint64(n_angle_nodes, 6 , "Number of nodes in the angle space");


int main (int argc, char ** argv)
{
    google::InitGoogleLogging(argv[0]);

    FLAGS_logtostderr = 1;

    google::ParseCommandLineFlags(&argc, &argv, true);



    CHECK(argc == 2) << "please provide an argument";

    std::string datadb = argv[1];

    LOG(INFO) << "working on file " << datadb;

    if (!FLAGS_linear)
    {
        CalibratorManager man(argv);
        man.readFile(datadb);
        man.setSolverOptions();

        man.setUpProblem();

        std::cout << "n pars blocks " << man.getNumberOfParameterBlocks() << std::endl;

        for (int i =0 ; i < man.getNumberOfParameterBlocks(); ++i)
        {
            std::cout << "par block " << i << " has size " << man.getSizeOfParameterBlock(i) << std::endl;
        }


        man.solve();
        man.printFullReport();
        man.printAllParameters();

        man.savePrediction("/home/luca/test.txt");

    }


    else
    {

        LOG(INFO) << "calibrating a linear model";

        ISerializable::Ptr o =  spc::io::deserializeFromFile(datadb);

        CHECK(o != NULL) << "cannot deserialize the file";

        EigenTable::Ptr table_ = spcDynamicPointerCast<EigenTable> (o);

        table_ = table_->getWithStrippedNANs({FLAGS_distance_field_name,
                                              FLAGS_angle_field_name,
                                              FLAGS_intensity_field_name,
                                              FLAGS_intensity_std_field_name});



        Eigen::VectorXf d = table_->column(FLAGS_distance_field_name);
        Eigen::VectorXf i = table_->column(FLAGS_intensity_field_name);
        Eigen::VectorXf a = table_->column(FLAGS_angle_field_name);



        CHECK(d.size() != 0 ) << "looks like your data is empty or not properly loaded";

        Eigen::Matrix<float, -1, -1> points;


        if (FLAGS_only_distance)
        {
            points.resize(d.rows(), 1);
            points.col(0) = d;
        }
        else
        {
            points.resize(d.rows(), 2);
            points.col(0) = d;
            points.col(1) = a;
        }



        spc::RBFModelEstimator<float> estimator;
        estimator.setPoints(points);
        estimator.setLambda(FLAGS_lambda);

        estimator.setInputValues(i);

        estimator.getModel()->setPolyOrder(FLAGS_poly_order);
        estimator.getModel()->setSigma(FLAGS_sigma);


        CHECK(estimator.solveProblem()!= -1) << "cannot solve -- see log info please";


//        LinearCalibrator calibrator(datadb);


//        std::string outfname = "/home/luca/test_linear.txt";

//        SampledData * data_ptr = &calibrator.getData();

//        Eigen::VectorXf pred = calibrator.getPredictionForInput();

//        EigenTable::Ptr out (new EigenTable);
//        out->addNewComponent("distance", 1);
//        out->addNewComponent("angle", 1);
//        out->addNewComponent("intensity", 1);
//        out->addNewComponent("pred_intensity", 1);




//        out->resize(data_ptr->i_.size());

//        out->column("distance") = data_ptr->d_;
//        out->column("angle") = data_ptr->a_;
//        out->column("intensity") = data_ptr->i_.cast<float>();
//        out->column("pred_intensity") = pred.cast<float>();


//        spc::io::AsciiEigenTableWriter w;
//        w.setInput(out);
//        w.setOutputFilename(outfname);
//        w.setWriteHeaders(true);
//        w.write();

    }
    return 1;
}
