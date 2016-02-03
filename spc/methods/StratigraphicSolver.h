#ifndef STRATIGRAPHICSOLVER_H
#define STRATIGRAPHICSOLVER_H

//#include <spc/elements/StratigraphicPositionableElement.h>
#include <spc/elements/StratigraphicModelBase.h>
#include <spc/elements/StratigraphicConstrain.h>
#include <spc/elements/StratigraphicPositionableElement.h>
#include <spc/core/spc_eigen.h>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
namespace spc {

template <typename _Scalar, int NX = Dynamic, int NY = Dynamic>
struct FunctorSolver {
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    FunctorSolver()
        : m_inputs(InputsAtCompileTime)
        , m_values(ValuesAtCompileTime)
    {
    }
    FunctorSolver(int inputs, int values)
        : m_inputs(inputs)
        , m_values(values)
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

class StratigraphicSolver {
public:
    spcTypedefSharedPtrs(StratigraphicSolver)

        enum MODE { NON_LINEAR = 0,
            LINEAR };

protected:
    struct my_functor : FunctorSolver<float> {
        my_functor(StratigraphicSolver* solver, int xsize, int ysize)
            : FunctorSolver<float>(xsize, ysize)
        {
            solver_ = solver;
        }

        int operator()(const VectorXf& x, VectorXf& fvec) const
        {
            LOG(INFO) << "CALLED functor, new pars: " << x.transpose();

            int counter = 0;
            for (spc::StratigraphicModelBase::Ptr mod : solver_->models_) {
                mod->setStratigraphicShift(x(counter++));
            }

            fvec = solver_->getSquaredResiduals();

            LOG(INFO) << "CALLED functor, residuals: " << fvec.transpose();

            return 0;
        }

        /// a pointer to the estimator itself
        StratigraphicSolver* solver_;
    };

public:
    StratigraphicSolver()
    {
    }

    /**
     * @brief reset restores the StratigraphicSolver as new
     */
    void reset();

    Eigen::VectorXf getSquaredResiduals() const;

    void solve();

    spcSetGetMacro(Mode, mode_, MODE);

    void setInputFromChildren(ElementBase::Ptr root_element);

protected:
    void solve_nonlinear();

    void solve_linear();

    std::vector<StratigraphicModelBase::Ptr> models_;
    std::vector<StratigraphicConstrain::Ptr> constrains_;
    std::vector<StratigraphicPositionableElement::Ptr> positionables_;

    MODE mode_ = LINEAR;
};
}

#endif // STRATIGRAPHICSOLVER_H
