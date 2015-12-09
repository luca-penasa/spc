#ifndef SPC_CORRECTION_FACTORS_H
#define SPC_CORRECTION_FACTORS_H

//#include <spc/ceres_calibration/SampleData.h>

#include <spc/elements/SamplesDB.h>
namespace spc
{

//!
//! \brief The CorrectionFactorBase class is the virtual base class for each
//! correction factor
//!
class CorrectionFactorBase
{
public:
    spcTypedefSharedPtrs(CorrectionFactorBase)
    EXPOSE_TYPE_BASE
    //! we pass a CalibrationSamplesDB because some factors may require to know
    //! some stuff before everything
    //! see for example CorrectionFactorCloudDependentMultiplier
    CorrectionFactorBase(SamplesDB::ConstPtr db) : is_fixed_(false)
    {
        // we extract here informations that may be of interest for the factors
        // - for inizialization etc
        n_clouds_ = db->getNumberOfDifferentClouds();
    }

    //! we force to give a name to the factor
    /** may be usefult in UI
     */
    virtual std::string getCorrectionName() const = 0;

    //! return a factor for the correction based on input data
    /** in most of the cases input data will be a vector with distance and angle
     * but it may contain other information used for predicting the factor
     */
    virtual float getFactor(Sample::ConstPtr input_data) const = 0;

    //! get suggested initialization parameters for this correction factor
    virtual Eigen::VectorXf getInitParameters() const = 0;

    virtual Eigen::VectorXf getParameters()
    {
        return parameters_;
    }

    //! get the number of expected parameters
    virtual size_t getNumberOfParameters() const
    {
        return this->getInitParameters().size();
    }

    virtual void setParameters(const Eigen::VectorXf &pars)
    {
        if (!is_fixed_)
            parameters_ = pars;
    }

    virtual void initParameters()
    {
        this->setParameters(this->getInitParameters());
    }

    // if called before initParameters(); you get a segfault.
    virtual void setLock(const bool is_fixed = true)
    {
        is_fixed_ = is_fixed;
    }

protected:
    Eigen::VectorXf parameters_;

    size_t n_clouds_;

    bool is_fixed_;
};

///
/// \brief The CorrectionFactorDistanceExp class is a correction for distance
/// based on D^a with a parameter
///
class CorrectionFactorDistancePowerLaw : public CorrectionFactorBase
{
public:
    CorrectionFactorDistancePowerLaw(SamplesDB::ConstPtr db)
        : CorrectionFactorBase(db)
    {
    }

    virtual std::string getCorrectionName() const
    {
        return "R^k";
    }

    virtual float getFactor(Sample::ConstPtr input_data) const
    {
        return pow(input_data->variantPropertyValue<float>("distance"), parameters_(0));
    }

    //! get suggested initialization parameters for this correction factor
    virtual Eigen::VectorXf getInitParameters() const
    {
        Eigen::VectorXf p(1);
        p(0) = -2.0;
        return p;
    }
};

//! cos-law in an exponential form
class CorrectionFactorCosPowerAngle : public CorrectionFactorBase
{
public:
    CorrectionFactorCosPowerAngle(SamplesDB::ConstPtr db)
        : CorrectionFactorBase(db)
    {
    }

    virtual std::string getCorrectionName() const
    {
        return "cos^k(angle)";
    }

    virtual float getFactor(Sample::ConstPtr input_data) const
    {
        float cosangle = cos(input_data->variantPropertyValue<float>("angle") / 180 * M_PI);
        return pow(cosangle, parameters_(0));
    }

    //! get suggested initialization parameters for this correction factor
    virtual Eigen::VectorXf getInitParameters() const
    {
        Eigen::VectorXf p(1);
        p(0) = 1.0f;
        return p;
    }
};

//! a simple multiplier in exp-form
class CorrectionFactorFixedMultiplier : public CorrectionFactorBase
{
public:
    CorrectionFactorFixedMultiplier(SamplesDB::ConstPtr db)
        : CorrectionFactorBase(db)
    {
    }

    virtual std::string getCorrectionName() const
    {
        return "exp(k)";
    }

    virtual float getFactor(Sample::ConstPtr input_data) const
    {
        return parameters_(0);
    }

    //! get suggested initialization parameters for this correction factor
    virtual Eigen::VectorXf getInitParameters() const
    {
        Eigen::VectorXf p(1);
        p(0) = 1;
        return p;
    }
};

//! two way correction for atmosphere
class CorrectionFactorAtmosphericTwoWay : public CorrectionFactorBase
{
public:
    CorrectionFactorAtmosphericTwoWay(SamplesDB::ConstPtr db)
        : CorrectionFactorBase(db)
    {
    }

    virtual std::string getCorrectionName() const
    {
        return "exp(2kR)";
    }

    virtual float getFactor(Sample::ConstPtr input_data) const
    {
        float R = input_data->variantPropertyValue<float>("distance");
        return exp(2 * R * parameters_(0));
    }

    virtual Eigen::VectorXf getInitParameters() const
    {
        Eigen::VectorXf p(1);
        p(0) = 0.0;
        return p;
    }
};

//! multiplier dependent on the cloud it is applied on
class CorrectionFactorCloudDependentMultiplier : public CorrectionFactorBase
{
public:
    CorrectionFactorCloudDependentMultiplier(SamplesDB::ConstPtr db)
        : CorrectionFactorBase(db)
    {
    }

    virtual std::string getCorrectionName() const
    {
        return "exp(k_n) for n in {0, 1, ... , N}, with N number of different "
               "input clouds";
    }

    virtual float getFactor(Sample::ConstPtr input_data) const
    {
        int cloud_id = input_data->variantPropertyValue<int>("cloud_id");
        return parameters_(cloud_id);
    }

    virtual Eigen::VectorXf getInitParameters() const
    {
        Eigen::VectorXf p(n_clouds_);
        for (size_t i = 0; i < n_clouds_; ++i)
            p(i) = 1.0;
        return p;
    }
};

} // end nspace

#endif
