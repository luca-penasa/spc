#include <spc/calibration/RBFEvaluator.h>


#include "ceres/ceres.h"
#include "glog/logging.h"

#include <spc/io/element_io.h>
#include <spc/elements/EigenTable.h>

#include <spc/io/AsciiEigenTableWriter.h>

#include <spc/calibration/CalibrationFactors.h>

using namespace spc;
using Eigen::Matrix;

//template class spc::RBFEvaluator<ceres::Jet<double, 6>>;



//size_t BasicFactor::current_number_of_instances_ =0;


#include <spc/calibration/CalibrationManager.h>











int main (int argc, char ** argv)
{
    std::string datadb = "/home/luca/Desktop/calibration_db.spc";
    CalibratorManager man(argv);
    man.readFile(datadb);

    man.setSolverOptions();


    man.setUpFixedPars();
    man.setUpInitParametersBlocks();
    man.setUpProblem();
    man.solve();
    man.printFullReport();

    man.savePrediction("/home/luca/test.txt");


////////////////////////////////////////////////////////////////////////////////////////////

//    std::cout << "dist pars:" << std::endl;
//    for (int i =0; i <n_dist_pars ; ++i)
//    {
//        std::cout << dist_pars[i] << std::endl;
//    }

//    std::cout << "angle pars:" << std::endl;
//    for (int i =0; i <n_ang_pars; ++i)
//    {
//        std::cout << angle_pars[i] << std::endl;
//    }


    /////////////////////////////////////////////// recomputing

//    Eigen::VectorXd predicted(obs.size());

//    for (int j = 0; j < obs.size(); ++j)
//    {
//        predicted(j) = predict_intensities(obs.at(j), *fixed_pars,  &parameters[0]);
//    }

//    EigenTable::Ptr out (new EigenTable);
//    out->addNewComponent("distance", 1);
//    out->addNewComponent("angle", 1);
//    out->addNewComponent("intensity", 1);
//    out->addNewComponent("pred_intensity", 1);


//    out->resize(i.rows());

//    out->column("distance") = d;
//    out->column("angle") = a;
//    out->column("intensity") = i.cast<float>();
//    out->column("pred_intensity") = predicted.cast<float>();


//    spc::io::AsciiEigenTableWriter w;
//    w.setInput(out);
//    w.setOutputFilename("/home/luca/test.txt");
//    w.setWriteHeaders(true);
//    w.write();

    return 1;
}
