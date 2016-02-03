#include "StratigraphicSolver.h"
namespace spc
{

void StratigraphicSolver::reset()
{
    models_.clear();
    constrains_.clear();
    positionables_.clear();
}

VectorXf StratigraphicSolver::getSquaredResiduals() const
{
    Eigen::VectorXf out;
    //        out.resize(constrains_.size() + positionables_.size());

    //        size_t i = 0;
    for (spc::StratigraphicConstrain::Ptr c: constrains_)
    {
        Eigen::VectorXf res = c->getSquaredResiduals();

        LOG(INFO) << "for constrain " << c->getElementName() << " " << res.transpose();
        out.conservativeResize(out.rows() + res.rows());
        out.tail(res.rows()) = res;


        LOG(INFO) << "out is: " << out.transpose();

    }

    for (StratigraphicPositionableElement::Ptr p: positionables_)
    {
        out.conservativeResize(out.rows() + 1);
        float res = p->getSquaredResidual();
        out.tail(1)(0) = res;
        LOG(INFO) << "residual for " << p->getElementName()   <<  res;
    }

    LOG(INFO) <<  "current residuals: \n" << out;

    return out;
}

void StratigraphicSolver::solve()
{
    if (mode_ == NON_LINEAR)
    {
        LOG(INFO) << "computing non-linear solution";
        solve_nonlinear();

    }
    else if (mode_ == LINEAR)
    {
        LOG(INFO) << "computing linear solution";
        solve_linear();
    }
}

void StratigraphicSolver::setInputFromChildren(ElementBase::Ptr root_element)
{

    LOG(INFO) << "going to extract inputs from childs";
    models_.clear();
    constrains_.clear();
    positionables_.clear();


    models_ = root_element->findElementsThatAre<spc::StratigraphicModelBase>(&spc::StratigraphicModelBase::Type);


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

void StratigraphicSolver::solve_nonlinear()
{
    VectorXf init_x(models_.size());

    int counter = 0;
    for (spc::StratigraphicModelBase::Ptr mod: models_)
    {
        init_x(counter++) = mod->getStratigraphicShift();
    }
    LOG(INFO) << "init x: " << init_x.transpose();

    int residual_size = this->getSquaredResiduals().rows();

    //        int outsize = constrains_.size() + positionables_.size();

    my_functor Functor(this, init_x.rows(), residual_size);


    Eigen::NumericalDiff<my_functor> numDiff(Functor);
    LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, float> lm(numDiff);
    lm.parameters.factor = 1;

    LOG(INFO) << "lev marq initialized";


    LOG(INFO) << "starting minimization";
    int info = lm.minimize(init_x);

    LOG(INFO) << "Done. Exit status: " << info;
}

void StratigraphicSolver::solve_linear()
{

}






}//end
