#ifndef INTENSITYCALIBRATOR_H
#define INTENSITYCALIBRATOR_H

#include <spc/elements/ICalModelFactors.h>
#include <spc/elements/SamplesDB.h>
#include <boost/current_function.hpp>
#include <unsupported/Eigen/NonLinearOptimization>

namespace spc
{

// Generic functor
template <typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct FunctorBase
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix
        <Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    int m_inputs, m_values;

    FunctorBase() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime)
    {
    }
    FunctorBase(int inputs, int values) : m_inputs(inputs), m_values(values)
    {
    }

    int inputs() const
    {
        return m_inputs;
    }
    int values() const
    {
        return m_values;
    }
};

class IntensityCalibrator
{
public:
    SPC_OBJECT(IntensityCalibrator)

    IntensityCalibrator();

    void setModel(const IntensityCalibrationModelBase::Ptr model)
    {
        model_ = model;
    }

    IntensityCalibrationModelBase::Ptr getModel() const
    {
        return model_;
    }

    void setCalibrationSamplesDB(SamplesDB::Ptr db)
    {
        assert(db);
        db_ = db;

        // we extract raw intensities
        observed_intensities_.resize(db->size());

        size_t counter = 0;
        spcForEachMacro(Sample::Ptr core, db->getSamplesDB())
        {
            observed_intensities_(counter++) = core->variantPropertyValue<float>("intensity");
        }
    }

    SamplesDB::Ptr getCalibrationSamplesDB() const
    {
        return db_;
    }

    Eigen::VectorXf getResidualsSimpleComparison()
    {
        pcl::console::print_debug("%s called\n", BOOST_CURRENT_FUNCTION);

        Eigen::VectorXf corr = model_->getPredictedIntensities(db_);
        //        Eigen::VectorXf ones = Eigen::VectorXf::Zero(corr.size());

        return observed_intensities_ - corr;
    }

    // this functor returns simply the difference between the observed and
    // predicted intensities
    struct FunctorSimple : FunctorBase<float>
    {
        FunctorSimple(IntensityCalibrator *calibrator)
        {
            calibrator_ = calibrator;
            pcl::console::print_debug("functor constructed\n");
        }

        int operator()(const Eigen::VectorXf &pars,
                       Eigen::VectorXf &residuals) const
        {
            calibrator_->getModel()->setParameters(pars);
            residuals = calibrator_->getResidualsSimpleComparison();

            std::cout << "called with pars: \n"
                      << calibrator_->getModel()->getParameters() << std::endl;

            float sum = residuals.array().square().sum();

            pcl::console::print_debug("functor operator () called\n");
            pcl::console::print_info("current sum of squares: %g\n", sum);
        }

        // n pars
        int inputs() const
        {
            return calibrator_->getModel()->getNumberOfParameters();
        }

        int values() const
        {
            return calibrator_->getCalibrationSamplesDB()->size();
        }

        IntensityCalibrator *calibrator_;
    };

    int optimize()
    {
        model_->initParameters(); // initialize parameters
        Eigen::VectorXf pars = model_->getParameters();

        std::cout << "init pars:\n " << pars << "\n" << std::endl;

        FunctorSimple functor(this);

        Eigen::NumericalDiff<FunctorSimple> numDiff(functor, 1e-2f);

        Eigen::LevenbergMarquardt
            <Eigen::NumericalDiff<FunctorSimple>, float> lm(numDiff);
        lm.parameters.maxfev = 1000000000000;

        //        std::cout <<  << std::endl;
        pcl::console::print_debug("starting minimization\n ");

        Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(pars);
        std::cout << "status: " << status << std::endl;

        std::cout << "x that minimizes the function: " << std::endl << pars
                  << std::endl;

        return 0;
    }

private:
    IntensityCalibrationModelBase::Ptr model_;

    SamplesDB::Ptr db_;

    Eigen::VectorXf observed_intensities_;
};

} // end nspace

#endif // INTENSITYCALIBRATOR_H
