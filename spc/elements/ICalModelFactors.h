#ifndef SPC_INTENSITY_CALIBRATRION_MODELS_H
#define SPC_INTENSITY_CALIBRATRION_MODELS_H

#include <spc/elements/ICalModelFactors.h>
#include <spc/elements/SamplesDB.h>
#include <spc/methods/spc_eigen.h>
#include <spc/elements/ICalFactors.h>

#include <pcl/console/print.h>
namespace spc
{

////! base class for any CalibrationModel is a virtual class
//class OptimizableScalarModel
//{
//public:
//    SPC_OBJECT(OptimizableScalarModel)

//    OptimizableScalarModel()
//    {
//    }

//    spcSetMacro(Parameters, parameters_, Eigen::VectorXf)
//    spcGetMacro(Parameters, parameters_, Eigen::VectorXf)


//    size_t getNumberOfParameters() const
//    {
//        return parameters_.cols();
//    }


//    virtual void initParameters(const Eigen::VectorXf &init_pars)
//    {
//        parameters_ = init_pars;
//    }

//    //! this method MUST be implemented
//    virtual float getPrediction(const Eigen::VectorXf & observation) = 0;

//    virtual Eigen::VectorXf getPrediction (const Eigen::MatrixXf & observations)
//    {
//        Eigen::VectorXf out(observations.rows());
//        for (int i = 0; i < observations.rows(); ++i)
//        {
//            out.row(i) = this->getPrediction(observations.row(i));
//        }

//        return out;
//    }

//protected:
//    Eigen::VectorXf parameters_;
//};




} // end nspace

#endif // INTENSITYCALIBRATRIONMODELS_H
