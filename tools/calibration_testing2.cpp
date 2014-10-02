

//#include "ceres/ceres.h"
//#include "glog/logging.h"

//#include <spc/io/element_io.h>
//#include <spc/elements/EigenTable.h>

//#include <spc/io/AsciiEigenTableWriter.h>

//#include <spc/calibration/CalibrationFactors.h>



#include <spc/calibration/CalibrationManager.h>



using namespace spc;
using Eigen::Matrix;








int main (int argc, char ** argv)
{
    std::string datadb = "/home/luca/Desktop/calibration_db.spc";
    CalibratorManager man(argv);
    man.readFile(datadb);

    man.setSolverOptions();


    man.setUpFixedPars();
    man.setUpInitParametersBlocks();
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




    return 1;
}
