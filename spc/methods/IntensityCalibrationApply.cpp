#include "IntensityCalibrationApply.h"
namespace spc
{
IntensityCalibrationApplier::IntensityCalibrationApplier()
{
}

int IntensityCalibrationApplier::compute()
{

    PCL_DEBUG("starting intensity calibration compute()\n");

    if (!cal_function_)
    {
        PCL_ERROR("Cannot Compute, calibration function void\n");
        return -1;
    }

    if (!cloud_->hasField(distance_field_name_))
    {
        PCL_DEBUG("Computing distances...\n");
        spc::PointCloudHelpers::computeDistanceFromSensor(cloud_, distance_field_name_);
    }

    if (!cloud_->hasField(angle_field_name_) && cal_function_->getInputSize() == 2)
    {
        PCL_DEBUG("No angles field found\n");

        // try to compute one, but we need normals
        if (!cloud_->hasField("normal_x"))
        {
            if (!cloud_with_normals_) // no cloud w normals provided? error and return
            {
                PCL_ERROR("Cannot find normals for computing angles (and angles not provided), please provide a cloud with normals or an additional cloud from which the normals will be read\n");
                return -1;
            }

            PCL_DEBUG("Transferring normals\n");

            //else transfer the normals
            spc::PointCloudHelpers::transferNormals(cloud_with_normals_, cloud_, max_distance_normals_);
        }

        PCL_DEBUG("Now computing scattering angles\n");

        // now compute the scattering angles
        spc::PointCloudHelpers::computeScatteringAngle(cloud_, angle_field_name_);
    }


    PCL_DEBUG("getting the fields...\n");

    std::vector<float> d = cloud_->getField(distance_field_name_);
    std::vector<float> a = cloud_->getField(angle_field_name_);
    std::vector<float> i = cloud_->getField(intensity_field_name_);

    if (d.empty() && i.empty()) {
        PCL_ERROR("distance and/or intensities not found\n");
        return -1;
    }

    if (a.empty() && (cal_function_->getInputSize() == 2)) {
        PCL_ERROR(
                    "angle data not found but calibration model require it\n");
        return -1;
    }

    // remap to eigen
    Eigen::Map<Eigen::VectorXf> d_eig(d.data(), d.size());
    Eigen::Map<Eigen::VectorXf> a_eig(a.data(), a.size());
    Eigen::Map<Eigen::VectorXf> i_eig(i.data(), i.size());

    Eigen::MatrixXf points;
    if (cal_function_->getInputSize() == 2) {
        points.resize(d.size(), 2);
        points.col(0) = d_eig;
        points.col(1) = (M_PI/180 * a_eig.array()).cos();
    } else {
        points.resize(d.size(), 1);
        points.col(0) = d_eig;
    }

    // clear the vectors we do not need anymore
    std::vector<float>().swap(d);
    std::vector<float>().swap(a);

    Eigen::VectorXf result = cal_function_->At(points);

    cloud_->addField("intensity_corrected");
    Eigen::VectorXf corrected;

//    std::cout << result << std::endl;

    if (method_ == DIVIDE)
        corrected = i_eig.array() / result.array();

    else if (method_ == SUBTRACT)
        corrected = i_eig.array() - result.array();

    for (int i = 0; i < i_eig.size(); ++i)
        cloud_->setFieldValue(i, "intensity_corrected", corrected(i) );
}
}
