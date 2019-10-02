#include "IntensityCalibratorRBF.h"


#include <spc/methods/RBFModelEstimator.hpp>
#include <spc/methods/IntensityCalibrationDataEstimator.h>
#include <spc/elements/calibration/DataHolder.h>
#include <spc/elements/calibration/KeyPoint.h>
#include <spc/elements/calibration/Observation.h>
#include <spc/elements/NewSpcPointCloud.h>


namespace spc
{

namespace calibration
{

IntensityCalibratorRBF::IntensityCalibratorRBF()
{
}

float
IntensityCalibratorRBF::computeWeight(Observation::Ptr per_cloud_data)
{
    float std = per_cloud_data->intensity_std;
    float er = per_cloud_data->getParent()->eigen_ratio;

    float w = 0;
    if (std != 0)
        w = 1/(std*std) * (1 - 3*er);
    else
        w =(1 - 3*er);
    //		LOG(INFO) << "w: " << w;
    return w;
}

int IntensityCalibratorRBF::calibrate()
{

    calibration_data_->ereaseInvalidObservations(use_angle);

    calibration_data_ = calibration_data_->getValidKeypoints();

    LOG(INFO) << "data has " << calibration_data_->getTotalNumberOfObservations() << " valid entries after cleaning";

    Eigen::VectorXi materials_ids = calibration_data_->getUniqueMaterials();
    LOG(INFO) << "materials (ids) found in file: " << materials_ids;
    for (int i = 0; i < materials_ids.rows(); ++i)
    {
        int mat_id = materials_ids(i);
        LOG(INFO) << "mat id " << mat_id << " has " << calibration_data_->getKeypointsOnMaterial(mat_id)->getTotalNumberOfObservations() << " valid entries";
    }

    calibration::DataHolder::Ptr init_data = calibration_data_->getKeypointsOnMaterial(init_set_material_id);

    LOG(INFO) << "Using " << init_data->getTotalNumberOfObservations() << " datapoints for initializing the problem";


    Eigen::Matrix<float, -1, -1> points;
    Eigen::Matrix<float, -1, 1> intensities, w;

    LOG(INFO) << "using angle: " << use_angle;
    LOG(INFO) << "using weights: " << use_weights;
    extractVariablesAsMatrix(init_data, points, intensities, w, use_angle, use_weights);

    //        LOG(INFO) << points.transpose();
    //        LOG(INFO) << intensities.transpose();


    Eigen::VectorXi n_splits;
    n_splits.push_back(n_splits_distance_);

    if (use_angle)
        n_splits.push_back(n_splits_angle_);

    if(intensity_shift != 0)
        intensities.array() += intensity_shift;

    spc::RBFModelEstimator<float> estimator;
    estimator.setPoints(points);
    estimator.setWeights(w);
    estimator.setInputValues(intensities);
    estimator.getModel()->setPolyOrder(poly_order_);

    LOG(INFO) << "Polynomial order: " << poly_order_;



    //we extract here the whole dataset, so to have an idea of the ranges of the
    // variables, these will be used to autosetScales and autosetNodes
    Eigen::Matrix<float, -1, -1> points_all;
    Eigen::Matrix<float, -1, 1> intensities_all;

    LOG(INFO) << "using angle: " << use_angle;

    extractVariablesAsMatrix(calibration_data_, points_all, intensities_all, w, use_angle, false); //! we dont need to get also the weights
    estimator.autosetScales(0, points_all);
    estimator.autosetNodes(n_splits, points_all);


    if (sigma_ == 0 )
        estimator.autosetSigma();
    else
        estimator.getModel()->setSigma(sigma_);

    estimator.setLambda(lambda_);
    estimator.getModel()->setPolyOrder(poly_order_);
    estimator.initProblem();


    /////////////////// NOW ALL THE ADDITIONAL CONSTRAINTS /////////////////////////////////////

    if (append_additional_materials_constraints)
    {
        for (int i = 0; i < materials_ids.rows(); ++i)
        {
            int mid =  materials_ids(i);
            if (mid == -1 | mid == init_set_material_id)
                continue;

            LOG(INFO) << "adding secondary constraints for material id " << mid ;

            //! for each of these materials we will add an additional set of contraints
            Eigen::Matrix<float, -1, -1> points;
            Eigen::Matrix<float, -1, 1> intensities;
            Eigen::Matrix<float, -1, 1> weights;

            extractVariablesAsMatrix(calibration_data_->getKeypointsOnMaterial(mid), points, intensities, weights, use_angle, use_weights);

            //                LOG(INFO) << points.transpose();
            //                LOG(INFO) << intensities.transpose();

            if(intensity_shift != 0)
                intensities.array() += intensity_shift;


            LOG(INFO) << "Added  " << points.rows() - 1 << " new constraints" ;

            if (additional_materials_weight != 1)
            {
                LOG(INFO) << "Overall weight for additional materials: " << additional_materials_weight;
                weights.array() *= additional_materials_weight;
            }



            estimator.appendEqualityConstrainForPoints(points, intensities, weights);
        }

    }

    else
    {
        LOG(WARNING) << "not using any constraints for additional materials";
    }



    if (append_undef_material_constraints)
    {
        DataHolder::Ptr other = calibration_data_->getKeypointsOnMaterial(-1);
        LOG(INFO) << "adding constraints for keypoints on unprecised materials";
        LOG(INFO) << "found " <<other->getTotalNumberOfObservations() << " entries";


        size_t  counter = 0;
        for(KeyPoint::Ptr kp: other->getData())
        {
            if (kp->getNumberOfObservations() <=1)
                continue;

            else
            {
                Eigen::Matrix<float, -1, -1> points;
                Eigen::Matrix<float, -1, 1> intensities, weights;

                extractVariablesAsMatrix(kp, points, intensities, weights,  use_angle, use_weights);

                if(intensity_shift != 0)
                    intensities.array() += intensity_shift;

                if (undef_material_weight != 1)
                {
                    LOG(INFO) << "Overall weight for additional materials: " << undef_material_weight;
                    weights.array() *= undef_material_weight;
                }


                estimator.appendEqualityConstrainForPoints(points, intensities, weights);
                counter++;
            }
        }

        LOG(INFO) << "added " << counter +1  << "additional constraints for undef materials";

    }

    else
    {
        LOG(WARNING) << "not using any constraints undefined materials keypoints";
    }

    CHECK(estimator.solveProblem() == 1) << "cannot solve -- see log info please";

    //        LOG(INFO) << estimator.getA() ;
    model_ = estimator.getModel();

    return 1;
}


void
IntensityCalibratorRBF::extractVariablesAsMatrix(KeyPoint::Ptr data,
                         Eigen::Matrix<float, -1, -1> &points,
                         Eigen::Matrix<float, -1, 1> &intensity,
                         Eigen::Matrix<float, -1, 1> &weights,
                         bool also_angle,
                         bool extract_weights
                         )
{
    points.resize(data->getNumberOfObservations(), 1);
    intensity.resize(data->getNumberOfObservations());

    if (extract_weights)
        weights.resize(data->getNumberOfObservations());

    if (also_angle)
        points.resize(Eigen::NoChange, 2);

    size_t counter = 0;
    for (Observation::Ptr cdata: data->observations)
    {
        points(counter, 0) = cdata->distance;
        intensity(counter) = cdata->intensity;

        if (also_angle)
            points(counter, 1) = cdata->angle;

        if (extract_weights)
            weights(counter) = computeWeight(cdata);

        counter++;
    }
}

void
IntensityCalibratorRBF::extractVariablesAsMatrix(DataHolder::Ptr holder,
                         Eigen::Matrix<float, -1, -1> &points,
                         Eigen::Matrix<float, -1, 1> &intensity,
                         Eigen::Matrix<float, -1, 1> &weights,
                         bool also_angle,
                         bool extract_weights)
{

    NewSpcPointCloud::Ptr ascloud = holder->asPointCloud();
    points.resize(ascloud->getNumberOfPoints(), 1);
    intensity.resize(ascloud->getNumberOfPoints());


    points.col(0) = ascloud->getFieldByName("distance");

    if (also_angle)
    {
        points.conservativeResize(Eigen::NoChange, 2);
        points.col(1) = ascloud->getFieldByName("angle");
    }


    intensity = ascloud->getFieldByName("intensity");


    if (extract_weights)
    {
        weights.resize(ascloud->getNumberOfPoints());

        size_t counter = 0;
        //! we got the fields but we also need to get the weights
        for (KeyPoint::Ptr k: holder->getData())
        {
            for (Observation::Ptr observation: k->observations)
            {
                float w = computeWeight(observation);
                weights(counter++) = w;
            }
        }
    }
}

}

}//end nspace
