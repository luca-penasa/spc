

//#include "ceres/ceres.h"
//#include "glog/logging.h"

//#include <spc/io/element_io.h>
//#include <spc/elements/EigenTable.h>

//#include <spc/io/AsciiEigenTableWriter.h>

//#include <spc/calibration/CalibrationFactors.h>



#include <spc/calibration/CalibrationManager.h>

#include <spc/calibration/LinearCalibrator.h>


using namespace spc;
using Eigen::Matrix;



bool linear  = true;
int main (int argc, char ** argv)
{
    std::string datadb = "/home/luca/Desktop/calibration_db.spc";

    if (!linear)
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
        std::cout << "--> Using a linear calibrator" << std::endl;
        LinearCalibrator calibrator(datadb);


        std::string outfname = "/home/luca/test_linear.txt";

        SampledData * data_ptr = &calibrator.getData();

        Eigen::VectorXf pred = calibrator.getPredictionForInput();

        EigenTable::Ptr out (new EigenTable);
        out->addNewComponent("distance", 1);
        out->addNewComponent("angle", 1);
        out->addNewComponent("intensity", 1);
        out->addNewComponent("pred_intensity", 1);




        out->resize(data_ptr->i_.size());

        out->column("distance") = data_ptr->d_;
        out->column("angle") = data_ptr->a_;
        out->column("intensity") = data_ptr->i_.cast<float>();
        out->column("pred_intensity") = pred.cast<float>();


        spc::io::AsciiEigenTableWriter w;
        w.setInput(out);
        w.setOutputFilename(outfname);
        w.setWriteHeaders(true);
        w.write();

    }
    return 1;
}
