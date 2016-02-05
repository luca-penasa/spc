#include "GenericFilter.h"
namespace spc
{

GenericFilter::GenericFilter()
{

}

void GenericFilter::setInput(std::vector<ElementBase::Ptr> inelements)
{
    inelements_ = inelements;
}

void GenericFilter::setInput(ElementBase::Ptr inobj)
{
    std::vector<spc::ElementBase::Ptr> inelements;
    inelements.push_back(inobj);
    inelements_ = inelements;
}

bool GenericFilter::canCompute() const
{

    LOG(INFO) << "startin can compute in spc genfil";

    InputRequirements req = this->getInputRequirements();

    LOG(INFO) << "got requs";

    if (inelements_.size() < req.n_elements)
    {
        LOG(INFO) << "selection not the right size";
        return false;
    }

    LOG(INFO) << "getting element from list";

    spc::ElementBase::Ptr  el = inelements_.at(0);

    LOG(INFO) << "got it";

    if  (!el )
    {
        LOG(INFO) << "This should not happen. null pointer";
        return false;
    }

    if (!el->isA(req.types.at(0)))
    {
        LOG(INFO) << "not of the right type";
        return false;
    }

    else
    {
        LOG(INFO) << "Done. Returning true";
        return true;
    }


}

std::vector<ElementBase::Ptr> GenericFilter::getOutput() const
{
 return outelements_;
}

} //end nspace
