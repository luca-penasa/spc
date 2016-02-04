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
                LOG(INFO) << "found a positionable: " << el->getElementName();
            }


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
    Eigen::MatrixXf A;
    Eigen::VectorXf b;

    // we are assuming the constrained models are in models_
    size_t n_models = models_.size();
    size_t n_constr = constrains_.size();
    size_t n_pos = positionables_.size();

    std::map<spc::StratigraphicModelBase::Ptr, size_t> ptr_to_id_map;

    size_t count = 0;
    for (auto mod: models_)
        ptr_to_id_map[mod] = count++;

    A.resize(n_pos + n_constr, n_models);
    b.resize(n_models);

    A.fill(0.0);


    size_t row_counter = 0;

    for (spc::StratigraphicPositionableElement::Ptr pos: positionables_)
    {

        float ps = pos->predictStratigraphicPositionFromModel();

        spc::StratigraphicModelBase::Ptr mymodel = pos->getStratigraphicModel();
        float shift = mymodel->getStratigraphicShift();

        float d = ps - shift;

        size_t id = ptr_to_id_map.at(mymodel);

        A(row_counter, id) = 1;
        b(row_counter) = -d;

        row_counter++;

    }


    for (spc::StratigraphicConstrain::Ptr cons: constrains_)
    {
        spc::StratigraphicPositionableElement::Ptr pos1 = cons->getVertices().at(0);
        spc::StratigraphicPositionableElement::Ptr pos2 = cons->getVertices().at(1);

        float sp1 = pos1->getStratigraphicPosition();
        float sp2 = pos2->getStratigraphicPosition();

        spc::StratigraphicModelBase::Ptr model1 = pos1->getStratigraphicModel();
        spc::StratigraphicModelBase::Ptr model2 = pos2->getStratigraphicModel();


        float shift1 = model1->getStratigraphicShift();
        float shift2 = model2->getStratigraphicShift();

        float d1 = sp1 - shift1;
        float d2 = sp2 - shift2;

        size_t id1 = ptr_to_id_map.at(model1);
        size_t id2 = ptr_to_id_map.at(model2);

        A(row_counter, id1) = 1;
        A(row_counter, id2) = -1;

        b(row_counter) = d2 - d1;

        row_counter++;


    }


    LOG(INFO) << "matrix a \n" << A;

    LOG(INFO) << "vector b \n" << b.transpose();

    VectorXf solution;
    solution.fill(spcNANMacro);

    if (A.rows() == A.cols())
    {
        LOG(INFO) << "Problem looks perfectly constrained. Goind to find an exact solution.";
        solution = A.colPivHouseholderQr().solve(b);
    }
    else if (A.rows() < A.cols())
    {
        LOG(ERROR) << "Cannot find a solution! The problem is under-constrained \n"
                      "Please consider adding more constrains to your outcrop";
    }
    else if (A.rows() > A.cols())
    {
        LOG(WARNING) << "The problem is over-constrained. This might be or not an intended behavior"
                        "Consider reducing the number of constrains";

        solution = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
    }

    LOG(INFO) << "solution " << solution.transpose();


    for (int i = 0; i < solution.rows(); ++i)
    {

        spc::StratigraphicModelBase::Ptr mod = models_.at(i);
        mod->setStratigraphicShift(solution(i));
    }
}






}//end
