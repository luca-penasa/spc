#include "IntensityCalibratorRBFNonLinear.h"



namespace spc
{
namespace calibration
{


IntensityCalibratorRBFNonLinear::IntensityCalibratorRBFNonLinear()
{
}

void IntensityCalibratorRBFNonLinear::setCalibrationData(DataHolder::Ptr data)
{

    all_keypoints_ = data;

    //! we split the data in "materials"
    Eigen::VectorXi materials = data->getUniqueMaterials();

    for (int i = 0; i < materials.rows(); i++)
    {
        int mat_id = materials(i);

        if (mat_id == -1) // -1 is unclassified stuff, each material will be made of just 1 keypoint
        {
            DataHolder::Ptr unclassified = data->getKeypointsOnMaterial(-1);
            for (KeyPoint::Ptr kpoint: unclassified->getData())
            {

                Material::Ptr newmat (new Material(0,  mat_id));
                newmat->appendKeypoint(kpoint);
                materials_.push_back(newmat);

                //					newmat->lock(); // keep locked for DEBUG
            }

        }

        else // all other materials are treated the same actually
        {
            DataHolder::Ptr keys = data->getKeypointsOnMaterial(mat_id);
            Material::Ptr newmat (new Material(0,  mat_id, *keys));

            if (mat_id == fixed_material_id_)
            {
                newmat->lock();
                newmat->setFactor(value_for_fixed_material_);
            }

            materials_.push_back(newmat);
        }
    }

}

void IntensityCalibratorRBFNonLinear::initMaterialsFactorFromCurrentModel()
{
    for (Material::Ptr mat: materials_)
    {
        if (mat->isLocked())
            continue;

        Eigen::VectorXf all_factors (mat->getTotalNumberOfObservations());
        int counter = 0;
        for (Observation::Ptr ob: mat->getAllObservations())
        {
            float prediction = model_->operator ()(ob->getAsEigenPoint());
            if (!std::isfinite(prediction) | (prediction == 0))
            {
                prediction = 1;
            }
            all_factors(counter ++) = ob->intensity / prediction;
        }

        float avg = all_factors.sum() / all_factors.rows();

        mat->setFactor(avg);

        LOG(INFO) << "material factor set to " <<avg;

    }
}

VectorXf IntensityCalibratorRBFNonLinear::getActiveMaterialFactors() const
{
    Eigen::VectorXf out;
    for (Material::Ptr mat: materials_)
    {
        if (!mat->isLocked()) // locked materials will not be optimized
            out.push_back(mat->getFactor());
    }
    return out;
}

void IntensityCalibratorRBFNonLinear::setMaterialFactors(const VectorXf factors)
{
    int counter = 0;
    for (Material::Ptr mat: materials_)
    {
        if(!mat->isLocked()) // only if the material is NON locked
            mat->setFactor(factors(counter++));


    }
}

void IntensityCalibratorRBFNonLinear::optimize()
{
    disableMaterialsWithLessThanObservations(3);
    initMaterialsFactorFromCurrentModel();
    int outsize = all_keypoints_->getTotalNumberOfObservations();

    Eigen::VectorXf factors = getActiveMaterialFactors();

    LOG(INFO) << "found " <<factors.rows() << " active materials";


    my_functor Functor(this, model_->getCoefficients().rows() + factors.size(), outsize);

    Eigen::NumericalDiff<my_functor> numDiff(Functor);
    LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, float> lm(numDiff);

    lm.parameters.maxfev = 400000;

    LOG(INFO) << "lev marq initialized";

    Eigen::VectorXf coeffs = model_->getCoefficients();


    LOG(INFO) << "init coeffs " << coeffs.transpose();
    LOG(INFO) << "init factors " << factors.transpose();

    coeffs.conservativeResize(coeffs.rows() + factors.rows());
    coeffs.tail(factors.rows()) = factors;

    //		return;

    LOG(INFO) << "starting minimization";
    int info = lm.minimize(coeffs);

    LOG(INFO) << "Done. Exit status: " << info;
}





}
} // end nspaces
