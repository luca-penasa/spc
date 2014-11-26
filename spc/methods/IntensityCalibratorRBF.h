#ifndef INTENSITYCALIBRATORRBF_H
#define INTENSITYCALIBRATORRBF_H
#include <spc/methods/RBFModelEstimator.hpp>
#include <spc/methods/IntensityCalibrationDataEstimator.h>


namespace spc
{
namespace calibration
{


class IntensityCalibratorRBF
{


public:
    IntensityCalibratorRBF();

    int checkTheInputData()
    {

        LOG(INFO) << "perfoming checks";

        if (!calibration_data_->hasField("intensity"))
        {
            LOG(WARNING) << "cannot find the intensity field into the data. Cannot proceed.";
            return -1;
        }
        else
        {
            LOG(INFO) << "inensity ok";
        }

        if (!calibration_data_->hasField("distance"))
        {
            LOG(WARNING) << "cannot find the distance field into the data. Cannot proceed.";
            return -1;
        }
        else
        {
            LOG(INFO) << "distance ok";
        }

        if (calibration_data_->hasField("material"))
        {
            LOG(INFO) << "material list present";
            LOG(INFO) << "following materials in data: " << calibration_data_->getFieldByName("material").unique();
            has_materials = true;
        }
        else
        {
            LOG(WARNING) << "no materials found";
            has_materials = false;
        }

        if (calibration_data_->hasField("angle"))
        {
            LOG(INFO) << "found angle data. Going to calibrate using the angle";
            has_angle = true;
        }
        else
        {
            LOG(WARNING) << "no angle data found";
            has_angle = false;
        }

        return 1;
    }

    void setCalibrationData(const NewSpcPointCloud::Ptr clouddata)
    {
        calibration_data_ = clouddata;
    }


    int calibrate()
    {

        int status = checkTheInputData();
        if (status != 1)
        {
            LOG(ERROR) << "some error i the input dataset, please see log";
            return -1;
        }

        // filter out nans from the dataset
        *calibration_data_ = calibration_data_->filterOutNans({"intensity", "distance"});


        LOG(INFO) << "nans filtered out";
        Eigen::VectorXf distance = calibration_data_->getFieldByName("distance");
        Eigen::VectorXf intensity = calibration_data_->getFieldByName("intensity");
        Eigen::VectorXi materials(distance.rows());
        Eigen::VectorXf angle(distance.rows());

        if (has_materials)
            materials = calibration_data_->getFieldByName("material").cast<int>();
        else
        {
            materials.fill(0);
            LOG(WARNING) << "considering the points as coming from the same material";
        }

        if (has_angle)
        {
            angle = calibration_data_->getFieldByName("angle");
        }

        std::vector<Eigen::VectorXf> predictors;
        predictors.push_back(distance);
        if (has_angle)
            predictors.push_back(angle);

        Eigen::MatrixXf points( intensity.rows(), predictors.size());
        for (int i = 0; i < predictors.size(); ++i)
        {
            points.col(i) = predictors.at(i);
        }

        predictors.clear();

        Eigen::VectorXi n_splits;
        n_splits.push_back(n_splits_distance_);

        if (has_angle)
            n_splits.push_back(n_splits_angle_);


        spc::RBFModelEstimator<float> estimator;
        estimator.setPoints(points);
        estimator.autosetScales(0);
        estimator.autosetNodes(n_splits);


        if (sigma_ == 0 )
            estimator.autosetSigma();
        else
            estimator.getModel()->setSigma(sigma_);

        estimator.setLambda(lambda_);

        estimator.setInputValues(intensity);

        estimator.getModel()->setPolyOrder(poly_order_);

        CHECK(estimator.solveProblem()!= -1) << "cannot solve -- see log info please";


        model_ = estimator.getModel();
        return 1;
    }


    RBFModel<float>::Ptr getModel() const
    {
        return model_;
    }


protected:

    //! we keep the calibration data in its cloud representation cause its easier to
    //! use and more portable (and a cloud representation can be easily created from
    //! the CalibrationDataHolder, but not vice-versa for now)
    NewSpcPointCloud::Ptr calibration_data_;


    bool has_angle;
    bool has_materials;

    size_t n_splits_angle_ = 4;
    size_t n_splits_distance_ = 4;

    size_t poly_order_ = 1;

    float lambda_ = 0.01;
    float sigma_ =0.0;

    RBFModel<float>::Ptr model_;

};





}
}
#endif // INTENSITYCALIBRATORRBF_H
