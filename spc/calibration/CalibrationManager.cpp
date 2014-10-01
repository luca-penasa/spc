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
}

void CalibratorManager::readFile(const std::string &fname)
{
    ISerializable::Ptr o =  spc::io::deserializeFromFile(fname);

    table_ = spcDynamicPointerCast<EigenTable> (o);

    table_ = table_->getWithStrippedNANs({"distance", "intensity", "angle", "intensity_std"});

    std::cout << "Following columns found:\n" << std::endl;
    for (int i = 0; i < table_->getNumberOfColumns(); ++i)
    {
        std::cout << table_->getColumnName(i) << std::endl;
    }

    obs_ = table2observations(*table_);

    d_ = table_->mat().col(table_->getColumnId("distance"));
    a_ = table_->mat().col(table_->getColumnId("angle"));
    i_ = table_->mat().col(table_->getColumnId("intensity"));



    cloud_ids_ = table_->mat().col(table_->getColumnId("cloud_id")).cast<int>();
    unique_ids_ = unique(cloud_ids_);


}

void CalibratorManager::solve()
{
    ceres::Solve(options_, &problem_, &summary_);
}

void CalibratorManager::savePrediction(const std::string outfname) const
{
    Eigen::VectorXd predicted(obs_.size());

    for (int j = 0; j < obs_.size(); ++j)
    {
        predicted(j) = predict_intensities(obs_.at(j), fixed_pars_,  &parameters_[0]);
    }

    EigenTable::Ptr out (new EigenTable);
    out->addNewComponent("distance", 1);
    out->addNewComponent("angle", 1);
    out->addNewComponent("intensity", 1);
    out->addNewComponent("pred_intensity", 1);


    out->resize(obs_.size());

    out->column("distance") = d_;
    out->column("angle") = a_;
    out->column("intensity") = i_.cast<float>();
    out->column("pred_intensity") = predicted.cast<float>();


    spc::io::AsciiEigenTableWriter w;
    w.setInput(out);
    w.setOutputFilename(outfname);
    w.setWriteHeaders(true);
    w.write();
}



}//end nspace
