#ifndef STRATIGRAPHICSOLVER_H
#define STRATIGRAPHICSOLVER_H

//#include <spc/elements/StratigraphicPositionableElement.h>
#include <spc/elements/StratigraphicModelBase.h>
#include <spc/elements/StratigraphicConstrain.h>
#include <spc/elements/StratigraphicPositionableElement.h>
#include <spc/core/spc_eigen.h>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
namespace spc
{


template <typename _Scalar, int NX = Dynamic, int NY = Dynamic> struct Functor
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

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime)
    {
    }
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values)
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







class StratigraphicSolver
{
public:

    spcTypedefSharedPtrs(StratigraphicSolver);

protected:
    struct my_functor : Functor<float>
    {
        my_functor(StratigraphicSolver * solver, int xsize, int ysize) : Functor<float>(xsize, ysize)
        {
            solver_ = solver;
        }

        int operator()(const VectorXf &x, VectorXf &fvec) const
        {
            LOG(INFO)<< "CALLED functor, new pars: " << x.transpose();

            int counter = 0;
            for (spc::StratigraphicModelBase::Ptr mod: solver_->models_)
            {

                mod->setStratigraphicShift(x(counter++));
            }


            fvec = solver_->getSquaredResiduals();

            LOG(INFO)<< "CALLED functor, residuals: " << fvec.transpose();

            return 0;
        }

        /// a pointer to the estimator itself
        StratigraphicSolver * solver_;
    };

public:

    StratigraphicSolver()
    {
    }


    void clear()
    {
        models_.clear();
        constrains_.clear();
        positionables_.clear();
    }

    Eigen::VectorXf getSquaredResiduals() const
    {
        Eigen::VectorXf out;
        out.resize(constrains_.size() + positionables_.size());

        size_t i = 0;
        for (spc::StratigraphicConstrain::Ptr c: constrains_)
        {
            out(i++) = c->getSquaredResidual();
        }

        for (StratigraphicPositionableElement::Ptr p: positionables_)
        {
            out(i++) = p->getSquaredResidual();
        }

        return out;
    }


    void solve()
    {
        VectorXf init_x(models_.size());

        int counter = 0;
        for (spc::StratigraphicModelBase::Ptr mod: models_)
        {
               init_x(counter++) = mod->getStratigraphicShift();
        }
        LOG(INFO) << "init x: " << init_x.transpose();


        int outsize = constrains_.size() + positionables_.size();

        my_functor Functor(this, init_x.rows(), outsize);

        Eigen::NumericalDiff<my_functor> numDiff(Functor);
        LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, float> lm(numDiff);

        LOG(INFO) << "lev marq initialized";


        LOG(INFO) << "starting minimization";
        int info = lm.minimize(init_x);

        LOG(INFO) << "Done. Exit status: " << info;
    }


    void addStratigraphicModel(spc::StratigraphicModelBase::Ptr model)
    {
        models_.push_back(model);
    }

    void extractInputFromChildrens()
    {

        LOG(INFO) << "going to extract inputs from childs";
        constrains_.clear();
        positionables_.clear();

        for(StratigraphicModelBase::Ptr mod: models_)
        {
            std::vector<spc::ElementBase::Ptr> pos = mod->findElementsThatAre(&spc::StratigraphicPositionableElement::Type);

            for (spc::ElementBase::Ptr el: pos)
            {
                spc::StratigraphicPositionableElement::Ptr p = spcDynamicPointerCast<spc::StratigraphicPositionableElement>(el);
                if (p->getManual()) //! < only elements set to manual can provide a constrain
                {
                    positionables_.push_back(p);
                }

                LOG(INFO) << "found a positionable: " << el->getElementName();

            }

            std::vector<spc::ElementBase::Ptr> constrains = mod->findElementsThatAre(&spc::StratigraphicConstrain::Type);


            for (ElementBase::Ptr el: constrains)
            {
                spc::StratigraphicConstrain::Ptr p = spcDynamicPointerCast<StratigraphicConstrain>(el);
                constrains_.push_back(p);

                LOG(INFO) << "found a constrain: " << el->getElementName();

            }


        }

        LOG(INFO) << "extraction done ";

    }





protected:
    std::vector<StratigraphicModelBase::Ptr> models_;
    std::vector<StratigraphicConstrain::Ptr> constrains_;
    std::vector<StratigraphicPositionableElement::Ptr> positionables_;







};

}

#endif // STRATIGRAPHICSOLVER_H
