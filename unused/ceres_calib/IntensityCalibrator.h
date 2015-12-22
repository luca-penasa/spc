#ifndef INTENSITYCALIBRATOR_H
#define INTENSITYCALIBRATOR_H

#include <spc/elements/PointCloudBase.h>
#include <spc/ceres_calibration/IntensityModelBase.h>
#include <iostream>


namespace spc
{



class IntensityCalibrator
{
public:
    IntensityCalibrator()
    {
    }

    void setInputCloud(PointCloudBase::Ptr cloud)
    {
        in_cloud_ = cloud;
    }

    void setInputModel(const IntensityModelBase::ConstPtr model)
    {
        model_ = model;
    }

    bool initFields()
    {
        // get required fields
        std::vector<FieldToFieldNameMapper::FIELD_ENUM> req_fields =  model_->getRequiredFields();

        std::cout << "model requires " << req_fields.size() << std::endl;

        // and get their mapping to strings so we can extract the fields from the cloud
        bool ok = field_mapper_(req_fields, field_names_);
        if (!ok)
            return false;

        size_t counter = 0;
        for (auto n: field_names_)
        {
            std::cout << "Field " << counter++ << ": "<< n << std::endl;
        }

        // now check if those fields exists into the cloud

        for (std::string fname: field_names_)
        {
            if (!in_cloud_->hasField(fname))
            {
                std::cout << "the cloud to correct does not have the fields required by the model. Check your field-name mapping?" << std::endl;
                return false;
            }
        }

        // check also if the intensity field exists or not
        std::string intensity_fname ;
        ok = field_mapper_(FieldToFieldNameMapper::INTENSITY, intensity_fname);
        if (!ok)
            return false;

        if (!in_cloud_->hasField(intensity_fname))
        {
            std::cout << "intensity not present in input cloud!" << std::endl;
            return false;
        }

        in_cloud_->getField(intensity_fname, intensity_);

        fields_.resize(in_cloud_->getNumberOfPoints(), field_names_.size());

        // read the actual fields from the cloud
        counter = 0;
        for (std::string fname: field_names_)
        {
            Eigen::VectorXf field;
            in_cloud_->getField(fname, field);

            fields_.col(counter++) = field;
        }

        return true;
    }



    bool predictIntensities()
    {
        if (!initFields())
            return false;

        predicted_.resize(in_cloud_->getNumberOfPoints());
        corrected_.resize(in_cloud_->getNumberOfPoints());

        for (int i = 0 ; i < fields_.rows(); ++i)
        {
            predicted_(i) = model_->getPredictedIntensity(fields_.row(i));
            if (predicted_(i) == 0.0)
                corrected_(i) = std::numeric_limits<float>::quiet_NaN();
            else
                corrected_(i) = intensity_(i) / predicted_(i);
        }


        return true;
    }

    bool writeInCloud()
    {
        // we save both predicted and corrected intensity
        in_cloud_->addField("intensity_pred");
        in_cloud_->addField("intensity_corr");

        for (int i = 0; i < intensity_.rows(); ++i)
        {
            in_cloud_->setFieldValue(i, "intensity_pred", predicted_(i));
            in_cloud_->setFieldValue(i, "intensity_corr", corrected_(i));
        }

        return true;
    }

protected:
    PointCloudBase::Ptr in_cloud_;

    FieldToFieldNameMapper field_mapper_;

    IntensityModelBase::ConstPtr model_;

    std::vector<std::string> field_names_;

    Eigen::MatrixXf fields_;
    Eigen::VectorXf intensity_;

    Eigen::VectorXf predicted_;
    Eigen::VectorXf corrected_;



};

}

#endif // INTENSITYCALIBRATOR_H
