#include "CalibrationManager.h"

namespace spc
{

CalibratorManager::CalibratorManager(char **argv)
{
    google::InitGoogleLogging(argv[0]);
}

void CalibratorManager::setSolverOptions()
{
    options_.max_num_iterations = 1000;
    options_.function_tolerance = 1e-6;
    options_.gradient_tolerance = 1e-6;
    options_.linear_solver_type = ceres::DENSE_QR;
    options_.minimizer_progress_to_stdout = true;
    options_.num_threads = 4;
    options_.num_linear_solver_threads = 4;
}

void CalibratorManager::readFile(const std::string &fname)
{

    samples_.readFile(fname);

}

void CalibratorManager::solve()
{
    ceres::Solve(options_, &problem_, &summary_);
}

void CalibratorManager::printFullReport()
{
    std::cout << summary_.FullReport() << "\n";
}

void CalibratorManager::printAllParameters( )
{

    BasicResidualBlock * resBlock = getPredictorBlock();



    for (ParameterBlock * b: resBlock->getMyBlocks())
    {

            std::cout << "o--> Parameter " << b->getBlockName() ;
            if(b->isEnabled())
                std::cout << " (OPT)" << std::endl;
            else
                std::cout << " (FIX)" << std::endl;
            std::cout << b->getData();

            std::cout << std::endl;

    }
}

void CalibratorManager::savePrediction(const std::string outfname) const
{
    Eigen::VectorXd predicted(samples_.obs_.size());

    for (int j = 0; j < samples_.obs_.size(); ++j)
    {
        predicted(j) = predict_intensities(samples_.obs_.at(j), *this->getPredictorBlock(), &this->getPredictorBlock()->getMyActiveParameters()[0]);
    }

    EigenTable::Ptr out (new EigenTable);
    out->addNewComponent("distance", 1);
    out->addNewComponent("angle", 1);
    out->addNewComponent("intensity", 1);
    out->addNewComponent("pred_intensity", 1);




    out->resize(samples_.obs_.size());

    out->column("distance") = samples_.d_;
    out->column("angle") = samples_.a_;
    out->column("intensity") = samples_.i_.cast<float>();
    out->column("pred_intensity") = predicted.cast<float>();


    spc::io::AsciiEigenTableWriter w;
    w.setInput(out);
    w.setOutputFilename(outfname);
    w.setWriteHeaders(true);
    w.write();
}



}//end nspace
