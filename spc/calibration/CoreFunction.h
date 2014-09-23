#ifndef COREFUNCTION_H
#define COREFUNCTION_H

#include <spc/elements/ElementBase.h>

namespace spc
{

//!
//! \brief The CoreFunction class it the core of the calibration model.
//! it predicts intensities based on distance and possibly scattering angle
//! other calibration variables may be added in a future so it is thougth to be
//! multidimensional
//!
//! \note this is a virtual class.
//!
class CoreFunction: public ElementBase
{
public:
    SPC_OBJECT(CoreFunction)
    EXPOSE_TYPE


    CoreFunction();

    virtual float getIntensity (const Eigen::VectorXf &vars) = 0;


    int setParameters(const Eigen::VectorXf &pars)
    {
        if (pars.rows() != n_pars_)
        {
            std::cout << "wrong parameter size in setparameters!" << std::cout;
            return -1;
        }

        parameters_ = pars;
    }


    Eigen::VectorXf getParameters() const
    {
        return parameters_;
    }

    size_t getNumberOfParameters () const
    {
        return n_pars_;
    }

    size_t getNumberOfVariables() const
    {
        return n_vars_;
    }


protected:

    //! \brief n_vars_ is the dimensionality of input variables.
    //! it is expected to be 2 in the most common case,
    //! like distance + angle compensation
    size_t n_vars_ = 0;

    //! is the number of parameters this model needs
    size_t n_pars_ = 0;


    //! the actual parameters
    Eigen::VectorXf parameters_;

};




}//end nspace

#endif // COREFUNCTION_H
