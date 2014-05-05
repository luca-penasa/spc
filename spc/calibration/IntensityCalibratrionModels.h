#ifndef SPC_INTENSITY_CALIBRATRION_MODELS_H
#define SPC_INTENSITY_CALIBRATRION_MODELS_H

#include <spc/common/common_includes.h>
#include <spc/calibration/CorrectionFactors.h>
#include <spc/calibration/CalibrationDataDB.h>

namespace spc
{

//! base class for any CalibrationModel is a virtual class
class IntensityCalibrationModelBase
{
public:

    typedef boost::shared_ptr<IntensityCalibrationModelBase> Ptr;
    typedef const boost::shared_ptr<IntensityCalibrationModelBase> ConstPtr;

    IntensityCalibrationModelBase() {}

    Eigen::VectorXf getParameters() const
    {
        return parameters_;
    }

    size_t getNumberOfParameters() const
    {
        return parameters_.cols();
    }

    virtual void setParameters (const Eigen::VectorXf & pars)
    {
        parameters_ = pars;
    }

    void setCalibrationData(CalibrationDataDB::ConstPtr db)
    {
        db_ = db;
    }

    virtual void initParameters(const Eigen::VectorXf &init_pars = Eigen::VectorXf()) = 0;

    //! this method MUST be implemented
    virtual float getOverallCorrectionFactor(const CorePointData::ConstPtr point) = 0;

    virtual Eigen::VectorXf getCorrectedIntensities(CalibrationDataDB::ConstPtr in_data);

    virtual Eigen::VectorXf getPredictedIntensities(CalibrationDataDB::ConstPtr in_data);



protected:
    Eigen::VectorXf parameters_;

    CalibrationDataDB::Ptr db_;
};


/** A model based on componible factors, see CorrectionFactors.h
 */
class IntensityCalibratrionModelFactorsBased: public IntensityCalibrationModelBase
{
public:

    typedef boost::shared_ptr<IntensityCalibratrionModelFactorsBased> Ptr;
    typedef const boost::shared_ptr<IntensityCalibratrionModelFactorsBased> ConstPtr;

    IntensityCalibratrionModelFactorsBased() {}

    void appendFactor(CorrectionFactorBase::Ptr factor)
    {
        factors_.push_back(factor);
    }

    virtual void setParameters (const Eigen::VectorXf & pars)
    {
        parameters_ = pars;
        writeParametersToFactors(parameters_);
    }


    /** you are responsable to pass the right number of pars here. No check will be made
     */
    virtual void initParameters(const Eigen::VectorXf &init_pars = Eigen::VectorXf())
    {

        pcl::console::print_debug("%i \n", init_pars.cols());

        if (init_pars.cols() == 1) // NOTE we will never have 1-dim parameters vecotr hopefully!
        {
            pcl::console::print_debug("initializin parameters because passed are empty\n");

            this->initParametersInFactors();
            this->readParametersFromFactors();
        }
        else
        {
            pcl::console::print_debug("initializing parameters with passed ones\n");

            parameters_ = init_pars;
        }

        std::cout << "Initialized parameters as \n" << parameters_ << "\n" << std::endl;
    }

    void initParametersInFactors()
    {
        BOOST_FOREACH (CorrectionFactorBase::Ptr fac, factors_)
        {
            fac->initParameters();
        }
    }

    void readParametersFromFactors()
    {
        size_t tot_n_pars = 0;
        BOOST_FOREACH(CorrectionFactorBase::Ptr fac, factors_)
        {
            tot_n_pars += fac->getNumberOfParameters();
        }

        Eigen::VectorXf par(tot_n_pars); // clean pars

        size_t counter = 0;
        BOOST_FOREACH (CorrectionFactorBase::Ptr fac, factors_)
        {
            Eigen::VectorXf v = fac->getParameters();

            std::cout << "par\n" << v << std::endl;

            par.segment(counter, v.size()) = v;

            counter  += v.size();

        }

        std::cout << par << std::endl;

        this->setParameters(par);
    }

    void writeParametersToFactors(const Eigen::VectorXf &pars)
    {
        int position_in_parameters = 0;
        BOOST_FOREACH (CorrectionFactorBase::Ptr fac, factors_)
        {
            size_t n_pars = fac->getNumberOfParameters();

            Eigen::VectorXf par = parameters_.segment(position_in_parameters, n_pars);

            fac->setParameters(par);

            position_in_parameters += n_pars;
        }
    }

    virtual float getOverallCorrectionFactor(const CorePointData::ConstPtr point)
    {
        float out = 1;
        BOOST_FOREACH (CorrectionFactorBase::Ptr fac, factors_)
        {
            out *= (fac->getFactor(point));
        }

        return out;
    }

    size_t getNumberOfFactors()const
    {
        return factors_.size();
    }



protected:

    std::vector<CorrectionFactorBase::Ptr> factors_;
};


//!
//! \brief The JutzyModel class is special calibration model based on the paper Jutzi, 2009
//! it is a specialization of IntensityCalibratrionModelFactorsBased
//!
class JutzyModel: public IntensityCalibratrionModelFactorsBased
{
public:

    typedef boost::shared_ptr<JutzyModel> Ptr;
    typedef const boost::shared_ptr<JutzyModel> ConstPtr;

    JutzyModel(CalibrationDataDB::ConstPtr db, const bool fixed_mul = true)
    {
        CorrectionFactorDistancePowerLaw::Ptr dist_f (new CorrectionFactorDistancePowerLaw(db));
        CorrectionFactorCosPowerAngle::Ptr ang_f (new CorrectionFactorCosPowerAngle(db));
        CorrectionFactorFixedMultiplier::Ptr mul_f;
        if (fixed_mul)
            mul_f = CorrectionFactorFixedMultiplier::Ptr (new CorrectionFactorFixedMultiplier(db));
        else
            mul_f = CorrectionFactorCloudDependentMultiplier::Ptr (new CorrectionFactorCloudDependentMultiplier(db));

        this->appendFactor(dist_f);
        this->appendFactor(ang_f);
        this->appendFactor(mul_f);

        this->initParametersInFactors();

    }
};


}//end nspace

#endif // INTENSITYCALIBRATRIONMODELS_H
