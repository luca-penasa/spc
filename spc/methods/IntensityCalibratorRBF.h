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



    void setCalibrationData(const CalibrationDataHolder::Ptr clouddata)
    {
        calibration_data_ = clouddata;
    }


    static  void
    extractVariablesAsMatrix(CalibrationDataHolder::Ptr holder, Eigen::Matrix<float, -1, -1> &points, Eigen::Matrix<float, -1, 1> &intensity, bool also_angle)

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
    }

    static void
    extractVariablesAsMatrix(CalibrationKeyPoint::Ptr data, Eigen::Matrix<float, -1, -1> &points, Eigen::Matrix<float, -1, 1> &intensity, bool also_angle)
    {
        points.resize(data->getNumberOfEntries(), 1);
        intensity.resize(data->getNumberOfEntries());
        if (also_angle)
        {
            points.resize(Eigen::NoChange, 2);
        }

        size_t counter = 0;
        for (PerCloudCalibrationData::Ptr cdata: data->per_cloud_data)
        {
            points(counter, 0) = cdata->distance;
            if (also_angle)
            {
                points(counter, 1) = cdata->angle;
            }

            intensity(counter) = cdata->intensity;
            counter++;
        }
    }


    int calibrate()
    {

        calibration_data_->ereaseInvalidPerCloudEntries(use_angle);

        calibration_data_ = calibration_data_->getValidKeypoints();

        LOG(INFO) << "data has " << calibration_data_->getTotalNumberOfEntries() << " valid entries after cleaning";

        Eigen::VectorXi materials_ids = calibration_data_->getDefinedMaterials();
        LOG(INFO) << "materials (ids) found in file: " << materials_ids;
        for (int i = 0; i < materials_ids.rows(); ++i)
        {
            int mat_id = materials_ids(i);
            LOG(INFO) << "mat id " << mat_id << " has " << calibration_data_->getKeypointsOnMaterial(mat_id)->getTotalNumberOfEntries() << " valid entries";
        }

        calibration::CalibrationDataHolder::Ptr init_data = calibration_data_->getKeypointsOnMaterial(init_set_material_id);

        LOG(INFO) << "Using " << init_data->getTotalNumberOfEntries() << " datapoints for initializing the problem";


        Eigen::Matrix<float, -1, -1> points;
        Eigen::Matrix<float, -1, 1> intensities;

        LOG(INFO) << "using angle: " << use_angle;
        extractVariablesAsMatrix(init_data, points, intensities, use_angle);

//        LOG(INFO) << points.transpose();
//        LOG(INFO) << intensities.transpose();


        Eigen::VectorXi n_splits;
        n_splits.push_back(n_splits_distance_);

        if (use_angle)
            n_splits.push_back(n_splits_angle_);



        Eigen::Matrix<float, -1, -1> points_all;
        Eigen::Matrix<float, -1, 1> intensities_all;

        LOG(INFO) << "using angle: " << use_angle;
        extractVariablesAsMatrix(calibration_data_, points_all, intensities_all, use_angle);


        spc::RBFModelEstimator<float> estimator;
        estimator.setPoints(points);
        estimator.autosetScales(0, points_all);
        estimator.autosetNodes(n_splits, points_all);


        if (sigma_ == 0 )
            estimator.autosetSigma();
        else
            estimator.getModel()->setSigma(sigma_);

        estimator.setLambda(lambda_);
        estimator.setInputValues(intensities);
        estimator.getModel()->setPolyOrder(poly_order_);
        estimator.initProblem();



        ///////////////////////////////
        /// set in the weights here ///
        ///////////////////////////////


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

                extractVariablesAsMatrix(calibration_data_->getKeypointsOnMaterial(mid), points, intensities, use_angle);

//                LOG(INFO) << points.transpose();
//                LOG(INFO) << intensities.transpose();


                LOG(INFO) << "Added  " << points.rows() - 1 << " new constraints" ;


                estimator.appendEqualityConstrainForPoints(points, intensities);
            }

        }

        else
        {
            LOG(WARNING) << "not using any constraints for additional materials";
        }



        if (append_undef_material_constraints)
        {
            CalibrationDataHolder::Ptr other = calibration_data_->getKeypointsOnMaterial(-1);
            LOG(INFO) << "adding constraints for keypoints on unprecised materials";
            LOG(INFO) << "found " <<other->getTotalNumberOfEntries() << " entries";


            size_t  counter = 0;
            for(CalibrationKeyPoint::Ptr kp: other->getData())
            {
                if (kp->getNumberOfEntries() <=1)
                    continue;

                else
                {
                    Eigen::Matrix<float, -1, -1> points;
                    Eigen::Matrix<float, -1, 1> intensities;

                    extractVariablesAsMatrix(kp, points, intensities, use_angle);

                    estimator.appendEqualityConstrainForPoints(points, intensities);
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


    RBFModel<float>::Ptr getModel() const
    {
        return model_;
    }

    spcSetMacro (UseAngle, use_angle, bool)
    spcSetMacro (UseWeights, use_weights, bool)
    spcSetMacro (AppendUndefMaterialsConstraints, append_undef_material_constraints, bool)
    spcSetMacro (AppendAdditionalMaterialsConstraints, append_additional_materials_constraints, bool)
    spcSetMacro (NAngleSplits, n_splits_angle_, size_t)
    spcSetMacro (NDistanceSplits, n_splits_distance_, size_t)

    spcSetMacro (Lambda, lambda_, float)
    spcSetMacro (ManualSigma, sigma_, float)

    spcSetMacro (InitSetMaterial, init_set_material_id, int)



protected:

    //! we keep the calibration data in its cloud representation cause its easier to
    //! use and more portable (and a cloud representation can be easily created from
    //! the CalibrationDataHolder, but not vice-versa for now)
    CalibrationDataHolder::Ptr calibration_data_;


    bool use_angle = true;
    bool use_weights = true;

    bool append_undef_material_constraints = true;
    bool append_additional_materials_constraints = true;

    bool has_materials;

    size_t n_splits_angle_ = 5;
    size_t n_splits_distance_ = 6;

    size_t poly_order_ = 1;

    float lambda_ = 0.00; //! < no lambda by def
    float sigma_ =0.0; //! < this will force the auto-mode by default


    int init_set_material_id = 0;

    RBFModel<float>::Ptr model_;

};





}
}
#endif // INTENSITYCALIBRATORRBF_H
